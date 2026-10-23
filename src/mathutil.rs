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
