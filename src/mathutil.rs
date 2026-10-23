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
    let f = py_float(v).ok_or_else(|| format!("could not convert string to float: '{}'", v))?;
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
    params: &indexmap::IndexMap<String, f64>,
    error_func: &dyn Fn(&indexmap::IndexMap<String, f64>) -> f64,
) -> indexmap::IndexMap<String, f64> {
    // Define potential changes
    let mut params = params.clone();
    let mut dp: indexmap::IndexMap<String, f64> = adj_params.iter().map(|p| (p.clone(), 1.0)).collect();
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
