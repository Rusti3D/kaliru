// The parts of scipy.signal (1.14.1) used by load_cell/sos_filter.py:
// butter(N, Wn, btype, fs, output="sos") for N <= 2, iirnotch, tf2sos and
// sosfilt_zi. Ported from scipy/signal/_filter_design.py and
// _signaltools.py, with numpy's complex arithmetic (Smith division) and
// OpenBLAS's LU for the 2x2 solve. LAPACK's eigenvalue routine behind
// np.roots is replaced by the quadratic formula, so results agree with
// scipy to rounding: they are checked through the fixed point values
// sos_filter makes of them (tools/gen_sos_cases.py).
use std::f64::consts::PI;

/// a numpy complex128
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct C64 {
    pub re: f64,
    pub im: f64,
}

impl C64 {
    pub const fn new(re: f64, im: f64) -> Self {
        C64 { re, im }
    }
    fn real(re: f64) -> Self {
        C64 { re, im: 0.0 }
    }
    fn add(self, o: C64) -> C64 {
        C64::new(self.re + o.re, self.im + o.im)
    }
    fn sub(self, o: C64) -> C64 {
        C64::new(self.re - o.re, self.im - o.im)
    }
    fn neg(self) -> C64 {
        C64::new(-self.re, -self.im)
    }
    fn conj(self) -> C64 {
        C64::new(self.re, -self.im)
    }
    fn mul(self, o: C64) -> C64 {
        C64::new(self.re * o.re - self.im * o.im, self.re * o.im + self.im * o.re)
    }
    /// numpy's complex division (Smith's method)
    fn div(self, o: C64) -> C64 {
        let (in1r, in1i, in2r, in2i) = (self.re, self.im, o.re, o.im);
        let (in2r_abs, in2i_abs) = (in2r.abs(), in2i.abs());
        if in2r_abs >= in2i_abs {
            if in2r_abs == 0.0 && in2i_abs == 0.0 {
                return C64::new(in1r / in2r_abs, in1i / in2r_abs);
            }
            let rat = in2i / in2r;
            let scl = 1.0 / (in2r + in2i * rat);
            C64::new((in1r + in1i * rat) * scl, (in1i - in1r * rat) * scl)
        } else {
            let rat = in2r / in2i;
            let scl = 1.0 / (in2i + in2r * rat);
            C64::new((in1r * rat + in1i) * scl, (in1i * rat - in1r) * scl)
        }
    }
    /// abs(): hypot
    fn abs(self) -> f64 {
        self.re.hypot(self.im)
    }
    /// np.exp of a complex (cexp)
    fn exp(self) -> C64 {
        let e = self.re.exp();
        C64::new(e * self.im.cos(), e * self.im.sin())
    }
    /// np.isreal
    fn is_real(self) -> bool {
        self.im == 0.0
    }
}

/// np.prod of complex values
fn prod(v: &[C64]) -> C64 {
    v.iter().fold(C64::real(1.0), |a, b| a.mul(*b))
}

/// buttap(N): (zeros, poles, gain) of an analog butterworth prototype
fn buttap(n: i64) -> (Vec<C64>, Vec<C64>, f64) {
    // m = np.arange(-N+1, N, 2); p = -np.exp(1j * pi * m / (2 * N))
    let p = (0..n)
        .map(|i| {
            let m = (-n + 1 + 2 * i) as f64;
            // 1j * pi * m: (0 * m, pi * m), then / (2 * N) as a complex
            let c = C64::new(0.0 * m, PI * m).div(C64::real((2 * n) as f64));
            c.exp().neg()
        })
        .collect();
    (vec![], p, 1.0)
}

fn lp2lp_zpk(z: &[C64], p: &[C64], k: f64, wo: f64) -> (Vec<C64>, Vec<C64>, f64) {
    let degree = p.len() as i32 - z.len() as i32;
    let w = C64::real(wo);
    let z_lp = z.iter().map(|z| w.mul(*z)).collect();
    let p_lp = p.iter().map(|p| w.mul(*p)).collect();
    (z_lp, p_lp, k * wo.powi(degree))
}

fn lp2hp_zpk(z: &[C64], p: &[C64], k: f64, wo: f64) -> (Vec<C64>, Vec<C64>, f64) {
    let degree = p.len() - z.len();
    let w = C64::real(wo);
    let mut z_hp: Vec<C64> = z.iter().map(|z| w.div(*z)).collect();
    let p_hp = p.iter().map(|p| w.div(*p)).collect();
    // If lowpass had zeros at infinity, inverting moves them to origin.
    z_hp.extend(std::iter::repeat_n(C64::real(0.0), degree));
    // Cancel out gain change caused by inversion
    let nz: Vec<C64> = z.iter().map(|z| z.neg()).collect();
    let np_: Vec<C64> = p.iter().map(|p| p.neg()).collect();
    let k_hp = k * prod(&nz).div(prod(&np_)).re;
    (z_hp, p_hp, k_hp)
}

fn bilinear_zpk(z: &[C64], p: &[C64], k: f64, fs: f64) -> (Vec<C64>, Vec<C64>, f64) {
    let degree = p.len() - z.len();
    let fs2 = C64::real(2.0 * fs);
    // Bilinear transform the poles and zeros
    let mut z_z: Vec<C64> = z.iter().map(|z| fs2.add(*z).div(fs2.sub(*z))).collect();
    let p_z = p.iter().map(|p| fs2.add(*p).div(fs2.sub(*p))).collect();
    // Any zeros that were at infinity get moved to the Nyquist frequency
    z_z.extend(std::iter::repeat_n(C64::real(-1.0), degree));
    // Compensate for gain change
    let a: Vec<C64> = z.iter().map(|z| fs2.sub(*z)).collect();
    let b: Vec<C64> = p.iter().map(|p| fs2.sub(*p)).collect();
    (z_z, p_z, k * prod(&a).div(prod(&b)).re)
}

/// np.poly(roots): the monic polynomial with these roots
fn poly(roots: &[C64]) -> Vec<C64> {
    let mut a = vec![C64::real(1.0)];
    for r in roots {
        let v = [C64::real(1.0), r.neg()];
        let mut c = vec![C64::real(0.0); a.len() + 1];
        for (i, ai) in a.iter().enumerate() {
            for (j, vj) in v.iter().enumerate() {
                c[i + j] = c[i + j].add(ai.mul(*vj));
            }
        }
        a = c;
    }
    a
}

/// zpk2tf's real coefficients (the roots here come in conjugate pairs)
fn zpk2tf(z: &[C64], p: &[C64], k: f64) -> (Vec<f64>, Vec<f64>) {
    let b = poly(z).iter().map(|c| C64::real(k).mul(*c).re).collect();
    let a = poly(p).iter().map(|c| c.re).collect();
    (b, a)
}

fn single_zpksos(z: &[C64], p: &[C64], k: f64) -> [f64; 6] {
    let mut sos = [0.0; 6];
    let (b, a) = zpk2tf(z, p, k);
    sos[3 - b.len()..3].copy_from_slice(&b);
    sos[6 - a.len()..6].copy_from_slice(&a);
    sos
}

const TOL: f64 = 100.0 * f64::EPSILON;

/// _cplxreal(z) concatenated: one of each conjugate pair, then the reals
fn cplxreal(z: &[C64]) -> Vec<C64> {
    // Sort by real part, magnitude of imaginary part
    let mut z = z.to_vec();
    z.sort_by(|a, b| a.re.partial_cmp(&b.re).unwrap_or(std::cmp::Ordering::Equal).then(a.im.abs().partial_cmp(&b.im.abs()).unwrap_or(std::cmp::Ordering::Equal)));
    let is_real = |c: &C64| c.im.abs() <= TOL * c.abs();
    let zr: Vec<C64> = z.iter().filter(|c| is_real(c)).map(|c| C64::real(c.re)).collect();
    if zr.len() == z.len() {
        return zr;
    }
    let zc_all: Vec<C64> = z.iter().filter(|c| !is_real(c)).copied().collect();
    let zp: Vec<C64> = zc_all.iter().filter(|c| c.im > 0.0).copied().collect();
    let zn: Vec<C64> = zc_all.iter().filter(|c| c.im < 0.0).copied().collect();
    // (runs of equal real parts are sorted by |imag|; pairs here are single)
    // Average out numerical inaccuracy in real vs imag parts of pairs
    let mut out: Vec<C64> = zp.iter().zip(&zn).map(|(p, n)| p.add(n.conj()).div(C64::real(2.0))).collect();
    out.extend(zr);
    out
}

/// argmin(abs(1 - abs(p))): the digital pole closest to the unit circle
fn idx_worst(p: &[C64]) -> usize {
    let mut best = 0;
    for i in 1..p.len() {
        if (1.0 - p[i].abs()).abs() < (1.0 - p[best].abs()).abs() {
            best = i;
        }
    }
    best
}

/// _nearest_real_complex_idx(fro, to, which)
fn nearest_idx(fro: &[C64], to: C64, which: &str) -> usize {
    let mut order: Vec<usize> = (0..fro.len()).collect();
    order.sort_by(|a, b| fro[*a].sub(to).abs().partial_cmp(&fro[*b].sub(to).abs()).unwrap_or(std::cmp::Ordering::Equal));
    match which {
        "any" => order[0],
        "real" => *order.iter().find(|i| fro[**i].is_real()).expect("real zero"),
        _ => *order.iter().find(|i| !fro[**i].is_real()).expect("complex zero"),
    }
}

/// zpk2sos(z, p, k) of a digital filter ("nearest" pairing)
fn zpk2sos(z: &[C64], p: &[C64], k: f64) -> Vec<[f64; 6]> {
    if z.is_empty() && p.is_empty() {
        return vec![[k, 0.0, 0.0, 1.0, 0.0, 0.0]];
    }
    let mut p = p.to_vec();
    let mut z = z.to_vec();
    // ensure we have the same number of poles and zeros
    while p.len() < z.len() {
        p.push(C64::real(0.0));
    }
    while z.len() < p.len() {
        z.push(C64::real(0.0));
    }
    let n_sections = p.len().max(z.len()).div_ceil(2);
    if p.len() % 2 == 1 {
        p.push(C64::real(0.0));
        z.push(C64::real(0.0));
    }
    let mut z = cplxreal(&z);
    let mut p = cplxreal(&p);

    let mut sos = vec![[0.0; 6]; n_sections];
    // Construct the system, reversing order so the "worst" are last
    for si in (0..n_sections).rev() {
        // Select the next "worst" pole
        let p1 = p.remove(idx_worst(&p));
        let real_p = p.iter().filter(|c| c.is_real()).count();
        let real_z = z.iter().filter(|c| c.is_real()).count();
        if p1.is_real() && real_p == 0 {
            // Special case (1): last remaining real pole
            let z1 = z.remove(nearest_idx(&z, p1, "real"));
            sos[si] = single_zpksos(&[z1, C64::real(0.0)], &[p1, C64::real(0.0)], 1.0);
        } else if p.len() + 1 == z.len() && !p1.is_real() && real_p == 1 && real_z == 1 {
            // Special case (2): there's one real pole and one real zero
            // left, and an equal number of poles and zeros to pair up.
            // We *must* pair with a complex zero
            let z1 = z.remove(nearest_idx(&z, p1, "complex"));
            sos[si] = single_zpksos(&[z1, z1.conj()], &[p1, p1.conj()], 1.0);
        } else {
            let p2 = if p1.is_real() {
                let preal: Vec<usize> = (0..p.len()).filter(|i| p[*i].is_real()).collect();
                let sub: Vec<C64> = preal.iter().map(|i| p[*i]).collect();
                p.remove(preal[idx_worst(&sub)])
            } else {
                p1.conj()
            };
            // find closest zero
            if !z.is_empty() {
                let z1 = z.remove(nearest_idx(&z, p1, "any"));
                if !z1.is_real() {
                    sos[si] = single_zpksos(&[z1, z1.conj()], &[p1, p2], 1.0);
                } else if !z.is_empty() {
                    let z2 = z.remove(nearest_idx(&z, p1, "real"));
                    sos[si] = single_zpksos(&[z1, z2], &[p1, p2], 1.0);
                } else {
                    sos[si] = single_zpksos(&[z1], &[p1, p2], 1.0);
                }
            } else {
                // no more zeros
                sos[si] = single_zpksos(&[], &[p1, p2], 1.0);
            }
        }
    }
    // put gain in first sos
    for v in sos[0][..3].iter_mut() {
        *v *= k;
    }
    sos
}

/// butter(N, Wn, btype, fs=fs, output="sos")
pub fn butter_sos(n: i64, wn: f64, highpass: bool, fs: f64) -> Result<Vec<[f64; 6]>, String> {
    let wn = wn / (fs / 2.0);
    if wn <= 0.0 {
        return Err("filter critical frequencies must be greater than 0".into());
    }
    let (z, p, k) = buttap(n);
    if wn >= 1.0 {
        return Err(format!(
            "Digital filter critical frequencies must be 0 < Wn < fs/2 (fs={} -> fs/2={})",
            crate::pyfmt::py_float_repr(fs),
            crate::pyfmt::py_float_repr(fs / 2.0)
        ));
    }
    // Pre-warp frequencies for digital filter design
    let fs2 = 2.0;
    let warped = 2.0 * fs2 * (PI * wn / fs2).tan();
    let (z, p, k) = if highpass { lp2hp_zpk(&z, &p, k, warped) } else { lp2lp_zpk(&z, &p, k, warped) };
    let (z, p, k) = bilinear_zpk(&z, &p, k, fs2);
    Ok(zpk2sos(&z, &p, k))
}

/// iirnotch(w0, Q, fs): (b, a)
pub fn iirnotch(w0: f64, q: f64, fs: f64) -> Result<([f64; 3], [f64; 3]), String> {
    let w0 = 2.0 * w0 / fs;
    // Checks if w0 is within the range
    if !(0.0..=1.0).contains(&w0) {
        return Err("w0 should be such that 0 < w0 < 1".into());
    }
    // Get bandwidth
    let bw = w0 / q;
    // Normalize inputs
    let bw = bw * PI;
    let w0 = w0 * PI;
    let beta = (bw / 2.0).tan();
    // Compute gain: formula 11.3.6 (p.575) from reference [1]
    let gain = 1.0 / (1.0 + beta);
    let b = [gain * 1.0, gain * (-2.0 * w0.cos()), gain * 1.0];
    let a = [1.0, -2.0 * gain * w0.cos(), 2.0 * gain - 1.0];
    Ok((b, a))
}

/// the roots of the quadratic c0 x^2 + c1 x + c2 (np.roots of 3 values)
fn roots2(c: &[f64; 3]) -> Vec<C64> {
    let (a, b, cc) = (c[0], c[1], c[2]);
    let disc = b * b - 4.0 * a * cc;
    if disc >= 0.0 {
        let s = disc.sqrt();
        vec![C64::real((-b + s) / (2.0 * a)), C64::real((-b - s) / (2.0 * a))]
    } else {
        let s = (-disc).sqrt();
        vec![C64::new(-b / (2.0 * a), s / (2.0 * a)), C64::new(-b / (2.0 * a), -s / (2.0 * a))]
    }
}

/// tf2sos(b, a) of a second order section
pub fn tf2sos(b: &[f64; 3], a: &[f64; 3]) -> Vec<[f64; 6]> {
    // tf2zpk: normalize, gain = b[0]
    let a0 = a[0];
    let bn = [b[0] / a0, b[1] / a0, b[2] / a0];
    let an = [a[0] / a0, a[1] / a0, a[2] / a0];
    let k = bn[0];
    let bm = [bn[0] / k, bn[1] / k, bn[2] / k];
    zpk2sos(&roots2(&bm), &roots2(&an), k)
}

/// np.linalg.solve of a 2x2 system (OpenBLAS getf2 + getrs)
fn solve2(a: [[f64; 2]; 2], b: [f64; 2]) -> [f64; 2] {
    let (mut a, mut b) = (a, b);
    // column 0: pivot by first max |a|
    let piv = if a[1][0].abs() > a[0][0].abs() { 1 } else { 0 };
    if piv == 1 {
        a.swap(0, 1);
        b.swap(0, 1);
    }
    let r = 1.0 / a[0][0];
    a[1][0] *= r;
    a[1][1] -= a[1][0] * a[0][1];
    // L*X = B (unit lower), then U*X = B
    b[1] -= b[0] * a[1][0];
    b[1] /= a[1][1];
    b[0] -= b[1] * a[0][1];
    b[0] /= a[0][0];
    b
}

/// lfilter_zi(b, a) of a section with a[0] == 1
fn lfilter_zi(b: &[f64], a: &[f64]) -> [f64; 2] {
    let (mut b, mut a) = (b.to_vec(), a.to_vec());
    if a[0] != 1.0 {
        // Normalize the coefficients so a[0] == 1.
        let a0 = a[0];
        b.iter_mut().for_each(|v| *v /= a0);
        a.iter_mut().for_each(|v| *v /= a0);
    }
    // IminusA = eye(2) - companion(a).T, companion first row -a[1:]/a[0]
    let c0 = -a[1] / (1.0 * a[0]);
    let c1 = -a[2] / (1.0 * a[0]);
    let iminus_a = [[1.0 - c0, 0.0 - 1.0], [0.0 - c1, 1.0 - 0.0]];
    let bb = [b[1] - a[1] * b[0], b[2] - a[2] * b[0]];
    // Solve zi = A*zi + B
    solve2(iminus_a, bb)
}

/// sosfilt_zi(sos)
pub fn sosfilt_zi(sos: &[[f64; 6]]) -> Vec<[f64; 2]> {
    let mut zi = vec![];
    let mut scale = 1.0;
    for section in sos {
        let (b, a) = (&section[..3], &section[3..]);
        let z = lfilter_zi(b, a);
        zi.push([scale * z[0], scale * z[1]]);
        // H(1): the gain at omega=0
        scale *= (b[0] + b[1] + b[2]) / (a[0] + a[1] + a[2]);
    }
    zi
}
