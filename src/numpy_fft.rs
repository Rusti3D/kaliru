// numpy.fft.rfft (numpy 2.2: pocketfft_hdronly.h rfftp, forward, fct=1)
// for the lengths shaper_calibrate uses: powers of two, so the radix 4
// and 2 passes. The twiddles are computed like sincos_2pibyn<double>.
use std::f64::consts::PI;

/// sincos_2pibyn(n)[idx]
struct SinCos2PiByN {
    n: usize,
    mask: usize,
    shift: usize,
    v1: Vec<(f64, f64)>,
    v2: Vec<(f64, f64)>,
}

impl SinCos2PiByN {
    fn calc(x: usize, n: usize, ang: f64) -> (f64, f64) {
        let mut x = x << 3;
        if x < 4 * n {
            // first half
            if x < 2 * n {
                // first quadrant
                if x < n {
                    return ((x as f64 * ang).cos(), (x as f64 * ang).sin());
                }
                return (((2 * n - x) as f64 * ang).sin(), ((2 * n - x) as f64 * ang).cos());
            }
            // second quadrant
            x -= 2 * n;
            if x < n {
                return (-(x as f64 * ang).sin(), (x as f64 * ang).cos());
            }
            return (-((2 * n - x) as f64 * ang).cos(), ((2 * n - x) as f64 * ang).sin());
        }
        x = 8 * n - x;
        if x < 2 * n {
            // third quadrant
            if x < n {
                return ((x as f64 * ang).cos(), -(x as f64 * ang).sin());
            }
            return (((2 * n - x) as f64 * ang).sin(), -((2 * n - x) as f64 * ang).cos());
        }
        // fourth quadrant
        x -= 2 * n;
        if x < n {
            return (-(x as f64 * ang).sin(), -(x as f64 * ang).cos());
        }
        (-((2 * n - x) as f64 * ang).cos(), -((2 * n - x) as f64 * ang).sin())
    }

    fn new(n: usize) -> Self {
        // Thigh(0.25L*pi/n): for a power of two n the long double division
        // is exact, so rounding it to double gives (pi/4 as double) / n
        let ang = (PI / 4.0) / n as f64;
        let nval = (n + 2) / 2;
        let mut shift = 1;
        while (1usize << shift) * (1usize << shift) < nval {
            shift += 1;
        }
        let mask = (1usize << shift) - 1;
        let mut v1 = vec![(1.0, 0.0); mask + 1];
        for (i, v) in v1.iter_mut().enumerate().skip(1) {
            *v = Self::calc(i, n, ang);
        }
        let mut v2 = vec![(1.0, 0.0); nval.div_ceil(mask + 1)];
        for (i, v) in v2.iter_mut().enumerate().skip(1) {
            *v = Self::calc(i * (mask + 1), n, ang);
        }
        SinCos2PiByN { n, mask, shift, v1, v2 }
    }

    fn get(&self, idx: usize) -> (f64, f64) {
        if 2 * idx <= self.n {
            let (x1, x2) = (self.v1[idx & self.mask], self.v2[idx >> self.shift]);
            return (x1.0 * x2.0 - x1.1 * x2.1, x1.0 * x2.1 + x1.1 * x2.0);
        }
        let idx = self.n - idx;
        let (x1, x2) = (self.v1[idx & self.mask], self.v2[idx >> self.shift]);
        (x1.0 * x2.0 - x1.1 * x2.1, -(x1.0 * x2.1 + x1.1 * x2.0))
    }
}

struct Factor {
    fct: usize,
    tw: Vec<f64>,
}

/// pocketfft's rfftp plan of a power of two length
struct Rfftp {
    length: usize,
    fact: Vec<Factor>,
}

impl Rfftp {
    fn new(length: usize) -> Self {
        let mut fact: Vec<Factor> = vec![];
        // factorize
        let mut len = length;
        while len.is_multiple_of(4) {
            fact.push(Factor { fct: 4, tw: vec![] });
            len >>= 2;
        }
        if len.is_multiple_of(2) {
            len >>= 1;
            // factor 2 should be at the front of the factor list
            fact.push(Factor { fct: 2, tw: vec![] });
            let last = fact.len() - 1;
            fact.swap(0, last);
        }
        assert!(len == 1, "rfft length {} is not a power of two", length);
        // comp_twiddle
        let twid = SinCos2PiByN::new(length);
        let mut l1 = 1;
        let nfact = fact.len();
        for (k, f) in fact.iter_mut().enumerate() {
            let ip = f.fct;
            let ido = length / (l1 * ip);
            if k < nfact - 1 {
                // last factor doesn't need twiddles
                f.tw = vec![0.0; (ip - 1) * (ido - 1)];
                for j in 1..ip {
                    for i in 1..=(ido - 1) / 2 {
                        let t = twid.get(j * l1 * i);
                        f.tw[(j - 1) * (ido - 1) + 2 * i - 2] = t.0;
                        f.tw[(j - 1) * (ido - 1) + 2 * i - 1] = t.1;
                    }
                }
            }
            l1 *= ip;
        }
        Rfftp { length, fact }
    }

    fn radf2(ido: usize, l1: usize, cc: &[f64], ch: &mut [f64], wa: &[f64]) {
        let wa_ = |x: usize, i: usize| wa[i + x * (ido - 1)];
        let c = |a: usize, b: usize, c: usize| cc[a + ido * (b + l1 * c)];
        let h = |a: usize, b: usize, c: usize| a + ido * (b + 2 * c);
        for k in 0..l1 {
            ch[h(0, 0, k)] = c(0, k, 0) + c(0, k, 1);
            ch[h(ido - 1, 1, k)] = c(0, k, 0) - c(0, k, 1);
        }
        if ido & 1 == 0 {
            for k in 0..l1 {
                ch[h(0, 1, k)] = -c(ido - 1, k, 1);
                ch[h(ido - 1, 0, k)] = c(ido - 1, k, 0);
            }
        }
        if ido <= 2 {
            return;
        }
        for k in 0..l1 {
            let mut i = 2;
            while i < ido {
                let ic = ido - i;
                // MULPM: (a+ib) = conj(c+id) * (e+if)
                let (wc, wd, e, f) = (wa_(0, i - 2), wa_(0, i - 1), c(i - 1, k, 1), c(i, k, 1));
                let tr2 = wc * e + wd * f;
                let ti2 = wc * f - wd * e;
                ch[h(i - 1, 0, k)] = c(i - 1, k, 0) + tr2;
                ch[h(ic - 1, 1, k)] = c(i - 1, k, 0) - tr2;
                ch[h(i, 0, k)] = ti2 + c(i, k, 0);
                ch[h(ic, 1, k)] = ti2 - c(i, k, 0);
                i += 2;
            }
        }
    }

    fn radf4(ido: usize, l1: usize, cc: &[f64], ch: &mut [f64], wa: &[f64]) {
        const HSQT2: f64 = std::f64::consts::FRAC_1_SQRT_2;
        let wa_ = |x: usize, i: usize| wa[i + x * (ido - 1)];
        let c = |a: usize, b: usize, c: usize| cc[a + ido * (b + l1 * c)];
        let h = |a: usize, b: usize, c: usize| a + ido * (b + 4 * c);
        for k in 0..l1 {
            // PM(tr1,CH(0,2,k),CC(0,k,3),CC(0,k,1))
            let tr1 = c(0, k, 3) + c(0, k, 1);
            ch[h(0, 2, k)] = c(0, k, 3) - c(0, k, 1);
            // PM(tr2,CH(ido-1,1,k),CC(0,k,0),CC(0,k,2))
            let tr2 = c(0, k, 0) + c(0, k, 2);
            ch[h(ido - 1, 1, k)] = c(0, k, 0) - c(0, k, 2);
            // PM(CH(0,0,k),CH(ido-1,3,k),tr2,tr1)
            ch[h(0, 0, k)] = tr2 + tr1;
            ch[h(ido - 1, 3, k)] = tr2 - tr1;
        }
        if ido & 1 == 0 {
            for k in 0..l1 {
                let ti1 = -HSQT2 * (c(ido - 1, k, 1) + c(ido - 1, k, 3));
                let tr1 = HSQT2 * (c(ido - 1, k, 1) - c(ido - 1, k, 3));
                ch[h(ido - 1, 0, k)] = c(ido - 1, k, 0) + tr1;
                ch[h(ido - 1, 2, k)] = c(ido - 1, k, 0) - tr1;
                ch[h(0, 3, k)] = ti1 + c(ido - 1, k, 2);
                ch[h(0, 1, k)] = ti1 - c(ido - 1, k, 2);
            }
        }
        if ido <= 2 {
            return;
        }
        let mulpm = |wc: f64, wd: f64, e: f64, f: f64| (wc * e + wd * f, wc * f - wd * e);
        for k in 0..l1 {
            let mut i = 2;
            while i < ido {
                let ic = ido - i;
                let (cr2, ci2) = mulpm(wa_(0, i - 2), wa_(0, i - 1), c(i - 1, k, 1), c(i, k, 1));
                let (cr3, ci3) = mulpm(wa_(1, i - 2), wa_(1, i - 1), c(i - 1, k, 2), c(i, k, 2));
                let (cr4, ci4) = mulpm(wa_(2, i - 2), wa_(2, i - 1), c(i - 1, k, 3), c(i, k, 3));
                let (tr1, tr4) = (cr4 + cr2, cr4 - cr2);
                let (ti1, ti4) = (ci2 + ci4, ci2 - ci4);
                let (tr2, tr3) = (c(i - 1, k, 0) + cr3, c(i - 1, k, 0) - cr3);
                let (ti2, ti3) = (c(i, k, 0) + ci3, c(i, k, 0) - ci3);
                ch[h(i - 1, 0, k)] = tr2 + tr1;
                ch[h(ic - 1, 3, k)] = tr2 - tr1;
                ch[h(i, 0, k)] = ti1 + ti2;
                ch[h(ic, 3, k)] = ti1 - ti2;
                ch[h(i - 1, 2, k)] = tr3 + ti4;
                ch[h(ic - 1, 1, k)] = tr3 - ti4;
                ch[h(i, 2, k)] = tr4 + ti3;
                ch[h(ic, 1, k)] = tr4 - ti3;
                i += 2;
            }
        }
    }

    /// exec(c, fct=1, r2hc=true): FFTpack order R0,R1,I1,...,Rn/2
    fn forward(&self, c: &mut Vec<f64>) {
        if self.length == 1 {
            return;
        }
        let nf = self.fact.len();
        let mut p1 = std::mem::take(c);
        let mut p2 = vec![0.0; self.length];
        let mut l1 = self.length;
        for k1 in 0..nf {
            let k = nf - k1 - 1;
            let ip = self.fact[k].fct;
            let ido = self.length / l1;
            l1 /= ip;
            if ip == 4 {
                Self::radf4(ido, l1, &p1, &mut p2, &self.fact[k].tw);
            } else {
                Self::radf2(ido, l1, &p1, &mut p2, &self.fact[k].tw);
            }
            std::mem::swap(&mut p1, &mut p2);
        }
        *c = p1;
    }
}

/// np.fft.rfft(x) of a power of two length: (re, im) of the n/2+1 bins
pub fn rfft(x: &[f64]) -> Vec<(f64, f64)> {
    rfft_plan(x.len())(x)
}

/// a reusable rfft of one length (pocketfft caches its plans)
pub fn rfft_plan(n: usize) -> impl Fn(&[f64]) -> Vec<(f64, f64)> {
    let plan = Rfftp::new(n);
    move |x: &[f64]| {
        let mut c = x.to_vec();
        plan.forward(&mut c);
        let nout = n / 2 + 1;
        let mut out = Vec::with_capacity(nout);
        out.push((c[0], 0.0));
        for i in 1..nout {
            let re = c[2 * i - 1];
            let im = if 2 * i < n { c[2 * i] } else { 0.0 };
            out.push((re, im));
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use crate::mathutil::numpy_tests::parse_hex;

    /// RK_FFT_CASES: tools/gen_fft_cases.py output
    #[test]
    fn fft_vs_numpy() {
        let Ok(path) = std::env::var("RK_FFT_CASES") else { return };
        let text = std::fs::read_to_string(path).unwrap();
        let mut bad = std::collections::BTreeMap::new();
        for line in text.lines() {
            let (lhs, rhs) = line.split_once(" = ").unwrap();
            let mut it = lhs.split_whitespace();
            let kind = it.next().unwrap();
            let args: Vec<&str> = it.collect();
            let want: Vec<f64> = rhs.split([' ', ',']).filter(|s| !s.is_empty()).map(parse_hex).collect();
            let got: Vec<f64> = match kind {
                "rfft" => {
                    let x: Vec<f64> = args.iter().map(|a| parse_hex(a)).collect();
                    super::rfft(&x).iter().flat_map(|(r, i)| [*r, *i]).collect()
                }
                "kaiser" => crate::mathutil::np_kaiser(args[0].parse().unwrap(), parse_hex(args[1])),
                _ => vec![crate::mathutil::np_i0(parse_hex(args[0]))],
            };
            let e = bad.entry(kind.to_string()).or_insert((0, 0));
            e.1 += 1;
            if got.len() != want.len() || got.iter().zip(&want).any(|(a, b)| a.to_bits() != b.to_bits()) {
                e.0 += 1;
                if e.0 <= 2 {
                    let n = got.iter().zip(&want).filter(|(a, b)| a.to_bits() != b.to_bits()).count();
                    eprintln!("{} mismatch ({} args): {} of {} values differ", kind, args.len(), n, want.len());
                }
            }
        }
        for (k, (b, n)) in &bad {
            eprintln!("{}: {}/{} mismatches", k, b, n);
        }
        assert!(bad.values().all(|(b, _)| *b == 0));
    }
}
