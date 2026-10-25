// Simple math helper functions (port of klippy/mathutil.py)
//
// Copyright (C) 2018-2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
//
// Ported so far: safe_float. Coordinate descent, matrix helpers and the
// background process helpers are ported with their first user.
use crate::pyfmt::py_float;

/// safe_float(v): float(v), rejecting nan and inf (python raises ValueError)
pub fn safe_float(v: &str) -> Result<f64, String> {
    let f = py_float(v).ok_or_else(|| format!("could not convert string to float: {}", crate::pyfmt::py_str_repr(v)))?;
    if f.is_nan() || f.is_infinite() {
        return Err(format!("{} is not a valid float", v));
    }
    Ok(f)
}

/// numpy.interp(x, xp, fp) for a scalar x (numpy/core/src/multiarray/
/// compiled_base.c arr_interp without precomputed slopes)
pub fn np_interp(x: f64, xp: &[f64], fp: &[f64]) -> f64 {
    let lenxp = xp.len();
    let (lval, rval) = (fp[0], fp[lenxp - 1]);
    if x.is_nan() {
        return x;
    }
    // binary_search_with_guess: the last j with xp[j] <= x, -1 below
    // xp[0], lenxp above xp[lenxp - 1]
    let j: isize = if x > xp[lenxp - 1] {
        lenxp as isize
    } else if x < xp[0] {
        -1
    } else {
        let (mut imin, mut imax) = (0usize, lenxp);
        while imin < imax {
            let imid = imin + ((imax - imin) >> 1);
            if x >= xp[imid] {
                imin = imid + 1;
            } else {
                imax = imid;
            }
        }
        imin as isize - 1
    };
    if j == -1 {
        return lval;
    }
    if j == lenxp as isize {
        return rval;
    }
    let j = j as usize;
    if j == lenxp - 1 {
        return fp[j];
    }
    if xp[j] == x {
        // Avoid potential non-finite interpolation
        return fp[j];
    }
    let slope = (fp[j + 1] - fp[j]) / (xp[j + 1] - xp[j]);
    // If we get nan in one direction, try the other
    let mut res = slope * (x - xp[j]) + fp[j];
    if res.is_nan() {
        res = slope * (x - xp[j + 1]) + fp[j + 1];
        if res.is_nan() && fp[j] == fp[j + 1] {
            res = fp[j];
        }
    }
    res
}

/// coordinate_descent(adj_params, params, error_func): params by name, in
/// order
pub fn coordinate_descent(
    adj_params: &[String],
    params: &crate::IndexMap<String, f64>,
    error_func: &dyn Fn(&crate::IndexMap<String, f64>) -> f64,
) -> crate::IndexMap<String, f64> {
    // Define potential changes
    let mut params = params.clone();
    let mut dp: crate::IndexMap<String, f64> = adj_params.iter().map(|p| (p.clone(), 1.0)).collect();
    // Calculate the error
    let mut best_err = error_func(&params);
    crate::logging::info(&format!("Coordinate descent initial error: {}", crate::pyfmt::py_float_repr(best_err)));

    let threshold = 0.00001;
    let mut rounds = 0;

    while crate::pyfmt::py_sum(dp.values().copied()) > threshold && rounds < 10000 {
        rounds += 1;
        for param_name in adj_params {
            let orig = params[param_name];
            params.insert(param_name.clone(), orig + dp[param_name]);
            let err = error_func(&params);
            if err < best_err {
                // There was some improvement
                best_err = err;
                dp[param_name] *= 1.1;
                continue;
            }
            params.insert(param_name.clone(), orig - dp[param_name]);
            let err = error_func(&params);
            if err < best_err {
                // There was some improvement
                best_err = err;
                dp[param_name] *= 1.1;
                continue;
            }
            params.insert(param_name.clone(), orig);
            dp[param_name] *= 0.9;
        }
    }
    crate::logging::info(&format!(
        "Coordinate descent best_err: {}  rounds: {}",
        crate::pyfmt::py_float_repr(best_err),
        rounds
    ));
    params
}

// ----------------------------------------------------------------------
// Trilateration
// ----------------------------------------------------------------------

/// a python exception of the math helpers: (class, message)
pub type MathError = (&'static str, &'static str);

fn py_sqrt(x: f64) -> Result<f64, MathError> {
    if x < 0.0 {
        return Err(("ValueError", "math domain error"));
    }
    Ok(x.sqrt())
}

fn py_div(a: f64, b: f64) -> Result<f64, MathError> {
    if b == 0.0 {
        return Err(("ZeroDivisionError", "float division by zero"));
    }
    Ok(a / b)
}

// Trilateration finds the intersection of three spheres.  See the
// wikipedia article for the details of the algorithm.
pub fn trilateration(sphere_coords: &[[f64; 3]; 3], radius2: &[f64; 3]) -> Result<[f64; 3], MathError> {
    let [sphere_coord1, sphere_coord2, sphere_coord3] = *sphere_coords;
    let s21 = matrix_sub(sphere_coord2, sphere_coord1);
    let s31 = matrix_sub(sphere_coord3, sphere_coord1);

    let d = py_sqrt(matrix_magsq(s21))?;
    let ex = matrix_mul(s21, py_div(1.0, d)?);
    let i = matrix_dot(ex, s31);
    let vect_ey = matrix_sub(s31, matrix_mul(ex, i));
    let ey = matrix_mul(vect_ey, py_div(1.0, py_sqrt(matrix_magsq(vect_ey))?)?);
    let ez = matrix_cross(ex, ey);
    let j = matrix_dot(ey, s31);

    let x = py_div(radius2[0] - radius2[1] + d.powf(2.0), 2.0 * d)?;
    let y = py_div(radius2[0] - radius2[2] - x.powf(2.0) + (x - i).powf(2.0) + j.powf(2.0), 2.0 * j)?;
    let z = -py_sqrt(radius2[0] - x.powf(2.0) - y.powf(2.0))?;

    let ex_x = matrix_mul(ex, x);
    let ey_y = matrix_mul(ey, y);
    let ez_z = matrix_mul(ez, z);
    Ok(matrix_add(sphere_coord1, matrix_add(ex_x, matrix_add(ey_y, ez_z))))
}

// Matrix helper functions for 3x1 matrices

pub fn matrix_cross(m1: [f64; 3], m2: [f64; 3]) -> [f64; 3] {
    [m1[1] * m2[2] - m1[2] * m2[1], m1[2] * m2[0] - m1[0] * m2[2], m1[0] * m2[1] - m1[1] * m2[0]]
}

pub fn matrix_dot(m1: [f64; 3], m2: [f64; 3]) -> f64 {
    m1[0] * m2[0] + m1[1] * m2[1] + m1[2] * m2[2]
}

pub fn matrix_magsq(m1: [f64; 3]) -> f64 {
    m1[0].powf(2.0) + m1[1].powf(2.0) + m1[2].powf(2.0)
}

pub fn matrix_add(m1: [f64; 3], m2: [f64; 3]) -> [f64; 3] {
    [m1[0] + m2[0], m1[1] + m2[1], m1[2] + m2[2]]
}

pub fn matrix_sub(m1: [f64; 3], m2: [f64; 3]) -> [f64; 3] {
    [m1[0] - m2[0], m1[1] - m2[1], m1[2] - m2[2]]
}

pub fn matrix_mul(m1: [f64; 3], s: f64) -> [f64; 3] {
    [m1[0] * s, m1[1] * s, m1[2] * s]
}

/// python's math.hypot(*coordinates): CPython's vector_norm (lossless
/// scaling, double-length squares and sums, a differential correction)
pub fn py_hypot(coords: &[f64]) -> f64 {
    let vec: Vec<f64> = coords.iter().map(|x| x.abs()).collect();
    let mut max = 0.0f64;
    let mut found_nan = false;
    for &x in &vec {
        found_nan |= x.is_nan();
        if x > max {
            max = x;
        }
    }
    vector_norm(vec, max, found_nan)
}

fn dl_fast_sum(a: f64, b: f64) -> (f64, f64) {
    let x = a + b;
    let z = x - a;
    let y = b - z;
    (x, y)
}

fn dl_mul(x: f64, y: f64) -> (f64, f64) {
    let z = x * y;
    let zz = x.mul_add(y, -z);
    (z, zz)
}

fn vector_norm(mut vec: Vec<f64>, max: f64, found_nan: bool) -> f64 {
    if max.is_infinite() {
        return max;
    }
    if found_nan {
        return f64::NAN;
    }
    if max == 0.0 || vec.len() <= 1 {
        return max;
    }
    let max_e = frexp_exp(max);
    if max_e < -1023 {
        // When max_e < -1023, ldexp(1.0, -max_e) would overflow.
        for v in vec.iter_mut() {
            *v /= f64::MIN_POSITIVE; // convert subnormals to normals
        }
        return f64::MIN_POSITIVE * vector_norm(vec, max / f64::MIN_POSITIVE, found_nan);
    }
    let scale = ldexp(1.0, -max_e);
    let (mut csum, mut frac1, mut frac2) = (1.0f64, 0.0f64, 0.0f64);
    for &v in &vec {
        let x = v * scale; // lossless scaling
        let pr = dl_mul(x, x); // lossless squaring
        let sm = dl_fast_sum(csum, pr.0); // lossless addition
        csum = sm.0;
        frac1 += pr.1; // lossy addition
        frac2 += sm.1; // lossy addition
    }
    let mut h = (csum - 1.0 + (frac1 + frac2)).sqrt();
    let pr = dl_mul(-h, h);
    let sm = dl_fast_sum(csum, pr.0);
    csum = sm.0;
    frac1 += pr.1;
    frac2 += sm.1;
    let x = csum - 1.0 + (frac1 + frac2);
    h += x / (2.0 * h); // differential correction
    h / scale
}

/// frexp(x).1 for a finite, non-zero x
fn frexp_exp(x: f64) -> i32 {
    let bits = x.to_bits();
    let exp = ((bits >> 52) & 0x7ff) as i32;
    if exp == 0 {
        // subnormal
        let m = bits & ((1u64 << 52) - 1);
        return -1022 - (m.leading_zeros() as i32 - 12) ;
    }
    exp - 1022
}

fn ldexp(x: f64, e: i32) -> f64 {
    x * 2f64.powi(e)
}

/// numpy's pairwise_sum of a contiguous float64 array (the add.reduce of
/// a 1-D array; numpy/_core/src/umath/loops_utils.h.src)
pub fn np_pairwise_sum(a: &[f64]) -> f64 {
    let n = a.len();
    if n < 8 {
        let mut res = 0.0;
        for v in a {
            res += v;
        }
        res
    } else if n <= 128 {
        let mut r = [a[0], a[1], a[2], a[3], a[4], a[5], a[6], a[7]];
        let mut i = 8;
        while i < n - (n % 8) {
            for j in 0..8 {
                r[j] += a[i + j];
            }
            i += 8;
        }
        let mut res = ((r[0] + r[1]) + (r[2] + r[3])) + ((r[4] + r[5]) + (r[6] + r[7]));
        // do non multiple of 8 rest
        while i < n {
            res += a[i];
            i += 1;
        }
        res
    } else {
        // divide by two but avoid non-multiples of unroll factor
        let mut n2 = n / 2;
        n2 -= n2 % 8;
        np_pairwise_sum(&a[..n2]) + np_pairwise_sum(&a[n2..])
    }
}

/// numpy.mean(a) of a 1-D float list
pub fn np_mean(a: &[f64]) -> f64 {
    np_pairwise_sum(a) / a.len() as f64
}

/// numpy.std(a) of a 1-D float list
pub fn np_std(a: &[f64]) -> f64 {
    let mean = np_mean(a);
    let sq: Vec<f64> = a.iter().map(|v| (v - mean) * (v - mean)).collect();
    (np_pairwise_sum(&sq) / a.len() as f64).sqrt()
}

/// numpy.mean(rows, axis=0): the reduction over rows is sequential, except
/// for a single column, which is contiguous and summed pairwise
pub fn np_mean_axis0(rows: &[Vec<f64>]) -> Vec<f64> {
    if rows[0].len() == 1 {
        let col: Vec<f64> = rows.iter().map(|r| r[0]).collect();
        return vec![np_mean(&col)];
    }
    let mut acc = rows[0].clone();
    for r in &rows[1..] {
        for (a, v) in acc.iter_mut().zip(r) {
            *a += v;
        }
    }
    acc.iter().map(|v| v / rows.len() as f64).collect()
}

/// numpy.std(rows, axis=0)
pub fn np_std_axis0(rows: &[Vec<f64>]) -> Vec<f64> {
    let mean = np_mean_axis0(rows);
    let sq: Vec<Vec<f64>> = rows.iter().map(|r| r.iter().zip(&mean).map(|(v, m)| (v - m) * (v - m)).collect()).collect();
    np_mean_axis0(&sq).iter().map(|v| v.sqrt()).collect()
}

/// numpy.cross(a, b) of 3-vectors
pub fn np_cross3(a: &[f64; 3], b: &[f64; 3]) -> [f64; 3] {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}

/// numpy.dot(a, b) of 3-vectors
pub fn np_dot3(a: &[f64; 3], b: &[f64; 3]) -> f64 {
    let mut dot = 0.0;
    for i in 0..3 {
        dot += b[i] * a[i];
    }
    dot
}

/// numpy.linalg.solve(a, b) for a 3x3 system: OpenBLAS dgesv, whose getf2
/// is a left-looking LU (dot products, reciprocal pivot scaling), then
/// dgetrs; matched against numpy (tools/gen_numpy_cases.py). Err on a
/// singular matrix.
pub fn np_solve3(a: [[f64; 3]; 3], b: [f64; 3]) -> Result<[f64; 3], MathError> {
    let (mut a, mut b) = (a, b);
    let n = 3;
    let mut ipiv = [0usize; 3];
    for j in 0..n {
        // Apply the previous interchanges to column j
        for i in 0..j {
            let ip = ipiv[i];
            if ip != i {
                let t = a[i][j];
                a[i][j] = a[ip][j];
                a[ip][j] = t;
            }
        }
        // Solve with the unit lower part for the upper part of column j
        for i in 1..j {
            let mut d = 0.0;
            for k in 0..i {
                d += a[i][k] * a[k][j];
            }
            a[i][j] -= d;
        }
        // Update the rest of column j (gemv as dot products)
        for i in j..n {
            let mut t = 0.0;
            for k in 0..j {
                t += a[i][k] * a[k][j];
            }
            a[i][j] -= t;
        }
        // Find pivot (iamax: first max |a|)
        let mut jp = j;
        for i in j + 1..n {
            if a[i][j].abs() > a[jp][j].abs() {
                jp = i;
            }
        }
        ipiv[j] = jp;
        let temp1 = a[jp][j];
        if temp1 == 0.0 {
            return Err(("LinAlgError", "Singular matrix"));
        }
        if jp != j {
            for k in 0..=j {
                let t = a[j][k];
                a[j][k] = a[jp][k];
                a[jp][k] = t;
            }
        }
        let r = 1.0 / temp1;
        for i in j + 1..n {
            a[i][j] *= r;
        }
    }
    // dgetrs: apply row interchanges to the right hand side
    for j in 0..n {
        if ipiv[j] != j {
            b.swap(j, ipiv[j]);
        }
    }
    // Solve L*X = B, overwriting B with X (unit lower)
    for k in 0..n {
        for i in k + 1..n {
            b[i] -= b[k] * a[i][k];
        }
    }
    // Solve U*X = B
    for k in (0..n).rev() {
        b[k] /= a[k][k];
        for i in 0..k {
            b[i] -= b[k] * a[i][k];
        }
    }
    Ok(b)
}

const I0A: [f64; 30] = [
    -4.41534164647933937950E-18,
    3.33079451882223809783E-17,
    -2.43127984654795469359E-16,
    1.71539128555513303061E-15,
    -1.16853328779934516808E-14,
    7.67618549860493561688E-14,
    -4.85644678311192946090E-13,
    2.95505266312963983461E-12,
    -1.72682629144155570723E-11,
    9.67580903537323691224E-11,
    -5.18979560163526290666E-10,
    2.65982372468238665035E-9,
    -1.30002500998624804212E-8,
    6.04699502254191894932E-8,
    -2.67079385394061173391E-7,
    1.11738753912010371815E-6,
    -4.41673835845875056359E-6,
    1.64484480707288970893E-5,
    -5.75419501008210370398E-5,
    1.88502885095841655729E-4,
    -5.76375574538582365885E-4,
    1.63947561694133579842E-3,
    -4.32430999505057594430E-3,
    1.05464603945949983183E-2,
    -2.37374148058994688156E-2,
    4.93052842396707084878E-2,
    -9.49010970480476444210E-2,
    1.71620901522208775349E-1,
    -3.04682672343198398683E-1,
    6.76795274409476084995E-1,
];

const I0B: [f64; 25] = [
    -7.23318048787475395456E-18,
    -4.83050448594418207126E-18,
    4.46562142029675999901E-17,
    3.46122286769746109310E-17,
    -2.82762398051658348494E-16,
    -3.42548561967721913462E-16,
    1.77256013305652638360E-15,
    3.81168066935262242075E-15,
    -9.55484669882830764870E-15,
    -4.15056934728722208663E-14,
    1.54008621752140982691E-14,
    3.85277838274214270114E-13,
    7.18012445138366623367E-13,
    -1.79417853150680611778E-12,
    -1.32158118404477131188E-11,
    -3.14991652796324136454E-11,
    1.18891471078464383424E-11,
    4.94060238822496958910E-10,
    3.39623202570838634515E-9,
    2.26666899049817806459E-8,
    2.04891858946906374183E-7,
    2.89137052083475648297E-6,
    6.88975834691682398426E-5,
    3.36911647825569408990E-3,
    8.04490411014108831608E-1,
];

fn chbevl(x: f64, vals: &[f64]) -> f64 {
    let mut b0 = vals[0];
    let mut b1 = 0.0;
    let mut b2 = 0.0;
    for v in &vals[1..] {
        b2 = b1;
        b1 = b0;
        b0 = x * b1 - b2 + v;
    }
    0.5 * (b0 - b2)
}

/// numpy.i0 (the cephes Chebyshev expansions)
pub fn np_i0(x: f64) -> f64 {
    let x = x.abs();
    if x <= 8.0 {
        x.exp() * chbevl(x / 2.0 - 2.0, &I0A)
    } else {
        x.exp() * chbevl(32.0 / x - 2.0, &I0B) / x.sqrt()
    }
}

/// numpy.kaiser(M, beta)
pub fn np_kaiser(m: usize, beta: f64) -> Vec<f64> {
    if m == 1 {
        return vec![1.0];
    }
    let mf = m as f64;
    let alpha = (mf - 1.0) / 2.0;
    let i0b = np_i0(beta);
    (0..m)
        .map(|n| {
            let r = (n as f64 - alpha) / alpha;
            np_i0(beta * (1.0 - r * r).sqrt()) / i0b
        })
        .collect()
}

/// numpy.linalg.lstsq(a, b, rcond=None)[0]: the minimum norm least squares
/// solution, with singular values <= eps * max(m, n) * s_max treated as
/// zero (LAPACK dgelsd's cutoff). This uses a one-sided Jacobi SVD, not
/// dgelsd's divide and conquer, so results agree with numpy to rounding
/// (~1e-12 relative), not bit for bit.
pub fn np_lstsq(a: &[Vec<f64>], b: &[f64]) -> Vec<f64> {
    let m = a.len();
    let n = a.first().map(|r| r.len()).unwrap_or(0);
    // Orthogonalize the columns of w (m >= n: a; else a transposed)
    let tall = m >= n;
    let (rows, cols) = if tall { (m, n) } else { (n, m) };
    let mut w: Vec<Vec<f64>> = (0..cols).map(|j| (0..rows).map(|i| if tall { a[i][j] } else { a[j][i] }).collect()).collect();
    let mut v: Vec<Vec<f64>> = (0..cols).map(|j| (0..cols).map(|i| (i == j) as i64 as f64).collect()).collect();
    let dot = |x: &[f64], y: &[f64]| x.iter().zip(y).map(|(a, b)| a * b).sum::<f64>();
    for _sweep in 0..100 {
        let mut rotated = false;
        for p in 0..cols {
            for q in p + 1..cols {
                let alpha = dot(&w[p], &w[p]);
                let beta = dot(&w[q], &w[q]);
                let gamma = dot(&w[p], &w[q]);
                if gamma == 0.0 || gamma.abs() <= f64::EPSILON * (alpha * beta).sqrt() {
                    continue;
                }
                rotated = true;
                let zeta = (beta - alpha) / (2.0 * gamma);
                let t = zeta.signum() / (zeta.abs() + (1.0 + zeta * zeta).sqrt());
                let c = 1.0 / (1.0 + t * t).sqrt();
                let s = c * t;
                for mat in [&mut w, &mut v] {
                    for k in 0..mat[p].len() {
                        let (xp, xq) = (mat[p][k], mat[q][k]);
                        mat[p][k] = c * xp - s * xq;
                        mat[q][k] = s * xp + c * xq;
                    }
                }
            }
        }
        if !rotated {
            break;
        }
    }
    // w's columns are s_i * u_i; v's are the matching singular vectors
    let sv: Vec<f64> = w.iter().map(|c| dot(c, c).sqrt()).collect();
    let smax = sv.iter().copied().fold(0.0, f64::max);
    let cutoff = f64::EPSILON * m.max(n) as f64 * smax;
    let mut x = vec![0.0; n];
    for i in 0..cols {
        if sv[i] <= cutoff {
            continue;
        }
        let s2 = sv[i] * sv[i];
        if tall {
            // a = sum u_i s_i v_i^T: x = sum v_i (w_i . b) / s_i^2
            let k = dot(&w[i], b) / s2;
            for (xj, vj) in x.iter_mut().zip(&v[i]) {
                *xj += vj * k;
            }
        } else {
            // a^T = sum w_i v_i^T: x = sum w_i (v_i . b) / s_i^2
            let k = dot(&v[i], b) / s2;
            for (xj, wj) in x.iter_mut().zip(&w[i]) {
                *xj += wj * k;
            }
        }
    }
    x
}

#[cfg(test)]
mod hypot_tests {
    /// RK_HYPOT_CASES: lines "a_hex b_hex hypot_hex" from the reference python
    #[test]
    fn hypot_vs_python() {
        let Ok(path) = std::env::var("RK_HYPOT_CASES") else { return };
        let text = std::fs::read_to_string(path).unwrap();
        let parse = |s: &str| -> f64 {
            let neg = s.starts_with('-');
            let s = s.trim_start_matches('-');
            if s == "0x0.0p+0" {
                return if neg { -0.0 } else { 0.0 };
            }
            let (mant, exp) = s.trim_start_matches("0x").split_once('p').unwrap();
            let (ip, fp) = mant.split_once('.').unwrap_or((mant, ""));
            let mut m = u64::from_str_radix(ip, 16).unwrap() as f64;
            let mut scale = 1.0 / 16.0;
            for c in fp.chars() {
                m += c.to_digit(16).unwrap() as f64 * scale;
                scale /= 16.0;
            }
            let v = m * 2f64.powi(exp.parse::<i32>().unwrap());
            if neg { -v } else { v }
        };
        let mut bad = 0;
        for line in text.lines() {
            let p: Vec<&str> = line.split_whitespace().collect();
            let (a, b, want) = (parse(p[0]), parse(p[1]), parse(p[2]));
            let got = super::py_hypot(&[a, b]);
            if got.to_bits() != want.to_bits() {
                bad += 1;
                eprintln!("hypot({:e}, {:e}) = {:e} want {:e}", a, b, got, want);
            }
        }
        assert_eq!(bad, 0);
    }
}

#[cfg(test)]
pub(crate) mod numpy_tests {
    use super::*;
    use crate::IndexMap;

    /// a python float.hex() value
    pub(crate) fn parse_hex(s: &str) -> f64 {
        match s {
            "inf" => return f64::INFINITY,
            "-inf" => return f64::NEG_INFINITY,
            "nan" => return f64::NAN,
            _ => {}
        }
        let neg = s.starts_with('-');
        let s = s.trim_start_matches('-');
        let (mant, exp) = s.trim_start_matches("0x").split_once('p').unwrap();
        let (ip, fp) = mant.split_once('.').unwrap_or((mant, ""));
        let mut m = u64::from_str_radix(ip, 16).unwrap() as f64;
        let mut scale = 1.0 / 16.0;
        for c in fp.chars() {
            m += c.to_digit(16).unwrap() as f64 * scale;
            scale /= 16.0;
        }
        let v = m * 2f64.powi(exp.parse::<i32>().unwrap());
        if neg {
            -v
        } else {
            v
        }
    }

    /// RK_NUMPY_CASES: tools/gen_numpy_cases.py output
    #[test]
    fn numpy_vs_python() {
        let Ok(path) = std::env::var("RK_NUMPY_CASES") else { return };
        let text = std::fs::read_to_string(path).unwrap();
        let mut bad: IndexMap<String, (usize, usize)> = IndexMap::default();
        for line in text.lines() {
            let (lhs, rhs) = line.split_once(" = ").unwrap();
            let mut it = lhs.split_whitespace();
            let kind = it.next().unwrap();
            let args: Vec<&str> = it.collect();
            let want: Vec<&str> = rhs.split_whitespace().collect();
            let got: Option<Vec<f64>> = match kind {
                "std1" | "mean1" => {
                    let v: Vec<f64> = args.iter().map(|a| parse_hex(a)).collect();
                    Some(vec![if kind == "std1" { np_std(&v) } else { np_mean(&v) }])
                }
                "std0" | "mean0" => {
                    let (m, n): (usize, usize) = (args[0].parse().unwrap(), args[1].parse().unwrap());
                    let flat: Vec<f64> = args[2..].iter().map(|a| parse_hex(a)).collect();
                    let rows: Vec<Vec<f64>> = (0..m).map(|i| flat[i * n..(i + 1) * n].to_vec()).collect();
                    Some(if kind == "std0" { np_std_axis0(&rows) } else { np_mean_axis0(&rows) })
                }
                "solve" => {
                    let f: Vec<f64> = args.iter().map(|a| parse_hex(a)).collect();
                    let a = [[f[0], f[1], f[2]], [f[3], f[4], f[5]], [f[6], f[7], f[8]]];
                    np_solve3(a, [f[9], f[10], f[11]]).ok().map(|r| r.to_vec())
                }
                "normal" => {
                    let f: Vec<f64> = args.iter().map(|a| parse_hex(a)).collect();
                    let cp = np_cross3(&[1.0, 0.0, f[0]], &[0.0, 1.0, f[1]]);
                    Some(vec![cp[0], cp[1], cp[2], np_dot3(&cp, &[f[3], f[4], f[2]])])
                }
                _ => panic!("unknown case {}", kind),
            };
            let ok = match (&got, want.as_slice()) {
                (None, ["err"]) => true,
                (Some(g), w) if w != ["err"] => {
                    g.len() == w.len() && g.iter().zip(w).all(|(g, w)| g.to_bits() == parse_hex(w).to_bits() || (g.is_nan() && parse_hex(w).is_nan()))
                }
                _ => false,
            };
            let e = bad.entry(kind.to_string()).or_insert((0, 0));
            e.1 += 1;
            if !ok {
                e.0 += 1;
                if e.0 <= 3 {
                    eprintln!("{} mismatch: {:?} want {}", kind, got, rhs);
                }
            }
        }
        for (k, (b, n)) in &bad {
            eprintln!("{}: {}/{} mismatches", k, b, n);
        }
        assert!(bad.values().all(|(b, _)| *b == 0));
    }
}
