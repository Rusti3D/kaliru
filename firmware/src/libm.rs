// The math functions of newlib's libm (4.4.0, the arm-none-eabi C library
// Kalico links; its fdlibm code, __OBSOLETE_MATH on soft float targets),
// so results are bit for bit those of the C firmware: powf (ef_pow.c),
// sqrtf (ef_sqrt.c), sin (s_sin.c, e_rem_pio2.c, k_rem_pio2.c, k_sin.c,
// k_cos.c) and what they use.
//
// ====================================================
// Copyright (C) 1993 by Sun Microsystems, Inc. All rights reserved.
//
// Developed at SunPro, a Sun Microsystems, Inc. business.
// Permission to use, copy, modify, and distribute this
// software is freely granted, provided that this notice
// is preserved.
// ====================================================
//
// (float versions: conversion to float by Ian Lance Taylor, Cygnus Support)
#![allow(clippy::excessive_precision, clippy::eq_op, clippy::zero_divided_by_zero)]

// GET_FLOAT_WORD / SET_FLOAT_WORD
fn fword(x: f32) -> i32 {
    x.to_bits() as i32
}
fn from_fword(w: i32) -> f32 {
    f32::from_bits(w as u32)
}
// GET_HIGH_WORD / GET_LOW_WORD / INSERT_WORDS
fn high(x: f64) -> i32 {
    (x.to_bits() >> 32) as i32
}
fn low(x: f64) -> u32 {
    x.to_bits() as u32
}
fn words(hi: i32, lo: u32) -> f64 {
    f64::from_bits(((hi as u32 as u64) << 32) | lo as u64)
}

// (std::fabs(float): gcc's __builtin_fabsf, the sign bit cleared)
pub fn fabsf(x: f32) -> f32 {
    f32::from_bits(x.to_bits() & 0x7fffffff)
}

fn copysignf(x: f32, y: f32) -> f32 {
    f32::from_bits((x.to_bits() & 0x7fffffff) | (y.to_bits() & 0x80000000))
}

fn copysign(x: f64, y: f64) -> f64 {
    f64::from_bits((x.to_bits() & !(1 << 63)) | (y.to_bits() & (1 << 63)))
}

// issignalingf_inline (math_config.h, IEEE_754_2008_SNAN)
fn issignalingf(x: f32) -> bool {
    2u32.wrapping_mul(x.to_bits() ^ 0x00400000) > 0xFF800000
}

// __math_oflowf / __math_uflowf (math_errf.c)
fn oflowf(sign: bool) -> f32 {
    let y = f32::from_bits(0x70000000); // 0x1p97f
    (if sign { -y } else { y }) * y
}
fn uflowf(sign: bool) -> f32 {
    let y = f32::from_bits(0x10000000); // 0x1p-95f
    (if sign { -y } else { y }) * y
}

/****************************************************************
 * sqrtf (ef_sqrt.c, wf_sqrt.c)
 ****************************************************************/

fn ieee754_sqrtf(x: f32) -> f32 {
    let mut ix = fword(x);
    let hx = (ix & 0x7fffffff) as u32;

    // take care of Inf and NaN
    if hx >= 0x7f800000 {
        return x * x + x; // sqrt(NaN)=NaN, sqrt(+inf)=+inf sqrt(-inf)=sNaN
    }
    // take care of zero and -ves
    if hx == 0 {
        return x; // sqrt(+-0) = +-0
    }
    if ix < 0 {
        return (x - x) / (x - x); // sqrt(-ve) = sNaN
    }

    // normalize x
    let mut m = ix >> 23;
    if hx < 0x00800000 {
        // subnormal x
        let mut i = 0;
        while ix & 0x00800000 == 0 {
            ix <<= 1;
            i += 1;
        }
        m -= i - 1;
    }
    m -= 127; // unbias exponent
    ix = (ix & 0x007fffff) | 0x00800000;
    if m & 1 != 0 {
        // odd m, double x to make it even
        ix += ix;
    }
    m >>= 1; // m = [m/2]

    // generate sqrt(x) bit by bit
    ix += ix;
    let (mut q, mut s) = (0i32, 0i32); // q = sqrt(x)
    let mut r: u32 = 0x01000000; // r = moving bit from right to left

    while r != 0 {
        let t = s + r as i32;
        if t <= ix {
            s = t + r as i32;
            ix -= t;
            q += r as i32;
        }
        ix += ix;
        r >>= 1;
    }

    // use floating add to find out rounding direction (round to
    // nearest: one-tiny >= one, one+tiny == one)
    if ix != 0 {
        q += q & 1;
    }
    ix = (q >> 1) + 0x3f000000;
    ix += m << 23;
    from_fword(ix)
}

pub fn sqrtf(x: f32) -> f32 {
    let z = ieee754_sqrtf(x);
    if x.is_nan() {
        return z;
    }
    if x < 0.0 {
        // sqrtf(negative): 0.0f/0.0f
        f32::NAN
    } else {
        z
    }
}

/****************************************************************
 * powf (ef_pow.c; the wf_pow.c wrapper returns its result as is)
 ****************************************************************/

// (the C constants: double literals converted to float)
const BP: [f32; 2] = [1.0, 1.5];
const DP_H: [f32; 2] = [0.0, 5.84960938e-01f64 as f32]; // 0x3f15c000
const DP_L: [f32; 2] = [0.0, 1.56322085e-06f64 as f32]; // 0x35d1cfdc
const TWO24F: f32 = 16777216.0; // 0x4b800000
// poly coefs for (3/2)*(log(x)-2s-2/3*s**3
const L1: f32 = 6.0000002384e-01f64 as f32; // 0x3f19999a
const L2: f32 = 4.2857143283e-01f64 as f32; // 0x3edb6db7
const L3: f32 = 3.3333334327e-01f64 as f32; // 0x3eaaaaab
const L4: f32 = 2.7272811532e-01f64 as f32; // 0x3e8ba305
const L5: f32 = 2.3066075146e-01f64 as f32; // 0x3e6c3255
const L6: f32 = 2.0697501302e-01f64 as f32; // 0x3e53f142
const P1: f32 = 1.6666667163e-01f64 as f32; // 0x3e2aaaab
const P2: f32 = -2.7777778450e-03f64 as f32; // 0xbb360b61
const P3: f32 = 6.6137559770e-05f64 as f32; // 0x388ab355
const P4: f32 = -1.6533901999e-06f64 as f32; // 0xb5ddea0e
const P5: f32 = 4.1381369442e-08f64 as f32; // 0x3331bb4c
const LG2: f32 = 6.9314718246e-01f64 as f32; // 0x3f317218
const LG2_H: f32 = 6.93145752e-01f64 as f32; // 0x3f317200
const LG2_L: f32 = 1.42860654e-06f64 as f32; // 0x35bfbe8c
const OVT: f32 = 4.2995665694e-08f64 as f32; // -(128-log2(ovfl+.5ulp))
const CP: f32 = 9.6179670095e-01f64 as f32; // 0x3f76384f =2/(3ln2)
const CP_H: f32 = 9.6179199219e-01f64 as f32; // 0x3f763800 =head of cp
const CP_L: f32 = 4.7017383622e-06f64 as f32; // 0x369dc3a0 =tail of cp_h
const IVLN2: f32 = 1.4426950216e+00f64 as f32; // 0x3fb8aa3b =1/ln2
const IVLN2_H: f32 = 1.4426879883e+00f64 as f32; // 0x3fb8aa00 =16b 1/ln2
const IVLN2_L: f32 = 7.0526075433e-06f64 as f32; // 0x36eca570 =1/ln2 tail

// the float with its low 12 bits cleared
fn trunc12(x: f32) -> f32 {
    from_fword(fword(x) & 0xfffff000u32 as i32)
}

pub fn powf(x: f32, y: f32) -> f32 {
    let hx = fword(x);
    let hy = fword(y);
    let mut ix = hx & 0x7fffffff;
    let iy = hy & 0x7fffffff;

    // y==zero: x**0 = 1
    if iy == 0 {
        if issignalingf(x) {
            return x + y;
        }
        return 1.0;
    }

    // x|y==NaN return NaN unless x==1 then return 1
    if ix > 0x7f800000 || iy > 0x7f800000 {
        if hx == 0x3f800000 && !issignalingf(y) {
            return 1.0;
        } else {
            return x + y;
        }
    }

    // determine if y is an odd int when x < 0
    // yisint = 0	... y is not an integer
    // yisint = 1	... y is an odd int
    // yisint = 2	... y is an even int
    let mut yisint = 0;
    if hx < 0 {
        if iy >= 0x4b800000 {
            yisint = 2; // even integer y
        } else if iy >= 0x3f800000 {
            let k = (iy >> 23) - 0x7f; // exponent
            let j = iy >> (23 - k);
            if (j << (23 - k)) == iy {
                yisint = 2 - (j & 1);
            }
        }
    }

    // special value of y
    if iy == 0x7f800000 {
        // y is +-inf
        if ix == 0x3f800000 {
            return 1.0; // +-1**+-inf = 1
        } else if ix > 0x3f800000 {
            // (|x|>1)**+-inf = inf,0
            return if hy >= 0 { y } else { 0.0 };
        } else {
            // (|x|<1)**-,+inf = inf,0
            return if hy < 0 { -y } else { 0.0 };
        }
    }
    if iy == 0x3f800000 {
        // y is  +-1
        return if hy < 0 { 1.0 / x } else { x };
    }
    if hy == 0x40000000 {
        return x * x; // y is  2
    }
    if hy == 0x3f000000 && hx >= 0 {
        // y is  0.5, x >= +0
        return ieee754_sqrtf(x);
    }

    let mut ax = fabsf(x);
    // special value of x
    if ix == 0x7f800000 || ix == 0 || ix == 0x3f800000 {
        let mut z = ax; // x is +-0,+-inf,+-1
        if hy < 0 {
            z = 1.0 / z; // z = (1/|x|)
        }
        if hx < 0 {
            if ((ix - 0x3f800000) | yisint) == 0 {
                z = (z - z) / (z - z); // (-1)**non-int is NaN
            } else if yisint == 1 {
                z = -z; // (x<0)**odd = -(|x|**odd)
            }
        }
        return z;
    }

    // (x<0)**(non-int) is NaN
    if (((hx as u32 >> 31).wrapping_sub(1)) | yisint as u32) == 0 {
        return (x - x) / (x - x);
    }

    let (t1, t2);
    // |y| is huge
    if iy > 0x4d000000 {
        // if |y| > 2**27
        // over/underflow if x is not close to one
        if ix < 0x3f7ffff4 {
            return if hy < 0 { oflowf(false) } else { uflowf(false) };
        }
        if ix > 0x3f800007 {
            return if hy > 0 { oflowf(false) } else { uflowf(false) };
        }
        // now |1-x| is tiny <= 2**-20, suffice to compute
        // log(x) by x-x^2/2+x^3/3-x^4/4
        let t = ax - 1.0; // t has 20 trailing zeros
        let w = (t * t) * (0.5 - t * (0.333333333333f64 as f32 - t * 0.25));
        let u = IVLN2_H * t; // ivln2_h has 16 sig. bits
        let v = t * IVLN2_L - w * IVLN2;
        t1 = trunc12(u + v);
        t2 = v - (t1 - u);
    } else {
        let mut n = 0;
        // take care subnormal number
        if ix < 0x00800000 {
            ax *= TWO24F;
            n -= 24;
            ix = fword(ax);
        }
        n += (ix >> 23) - 0x7f;
        let j = ix & 0x007fffff;
        // determine interval
        ix = j | 0x3f800000; // normalize ix
        let k: usize;
        if j <= 0x1cc471 {
            k = 0; // |x|<sqrt(3/2)
        } else if j < 0x5db3d7 {
            k = 1; // |x|<sqrt(3)
        } else {
            k = 0;
            n += 1;
            ix -= 0x00800000;
        }
        ax = from_fword(ix);

        // compute s = s_h+s_l = (x-1)/(x+1) or (x-1.5)/(x+1.5)
        let u = ax - BP[k]; // bp[0]=1.0, bp[1]=1.5
        let v = 1.0 / (ax + BP[k]);
        let s = u * v;
        let s_h = trunc12(s);
        // t_h=ax+bp[k] High
        let t_h = from_fword(((ix >> 1) | 0x20000000) + 0x0040000 + ((k as i32) << 21));
        let t_l = ax - (t_h - BP[k]);
        let s_l = v * ((u - s_h * t_h) - s_h * t_l);
        // compute log(ax)
        let mut s2 = s * s;
        let mut r = s2 * s2 * (L1 + s2 * (L2 + s2 * (L3 + s2 * (L4 + s2 * (L5 + s2 * L6)))));
        r += s_l * (s_h + s);
        s2 = s_h * s_h;
        let t_h = trunc12(3.0 + s2 + r);
        let t_l = r - ((t_h - 3.0) - s2);
        // u+v = s*(1+...)
        let u = s_h * t_h;
        let v = s_l * t_h + t_l * s;
        // 2/(3log2)*(s+...)
        let p_h = trunc12(u + v);
        let p_l = v - (p_h - u);
        let z_h = CP_H * p_h; // cp_h+cp_l = 2/(3*log2)
        let z_l = CP_L * p_h + p_l * CP + DP_L[k];
        // log2(ax) = (s+..)*2/(3*log2) = n + dp_h + z_h + z_l
        let t = n as f32;
        t1 = trunc12(((z_h + z_l) + DP_H[k]) + t);
        t2 = z_l - (((t1 - t) - DP_H[k]) - z_h);
    }

    let mut s: f32 = 1.0; // s (sign of result -ve**odd) = -1 else = 1
    if (((hx as u32 >> 31).wrapping_sub(1)) | (yisint - 1) as u32) == 0 {
        s = -1.0; // (-ve)**(odd int)
    }

    // split up y into y1+y2 and compute (y1+y2)*(t1+t2)
    let y1 = trunc12(y);
    let p_l = (y - y1) * t1 + y * t2;
    let mut p_h = y1 * t1;
    let mut z = p_l + p_h;
    let mut j = fword(z);
    let i = j & 0x7fffffff;
    if j > 0 {
        if i > 0x43000000 {
            return oflowf(s < 0.0); // overflow
        } else if i == 0x43000000 && p_l + OVT > z - p_h {
            return oflowf(s < 0.0); // overflow
        }
    } else if i > 0x43160000 {
        return uflowf(s < 0.0); // underflow
    } else if i == 0x43160000 && p_l <= z - p_h {
        return uflowf(s < 0.0); // underflow
    }
    // compute 2**(p_h+p_l)
    let mut k = (i >> 23) - 0x7f;
    let mut n = 0;
    if i > 0x3f000000 {
        // if |z| > 0.5, set n = [z+0.5]
        n = j + (0x00800000 >> (k + 1));
        k = ((n & 0x7fffffff) >> 23) - 0x7f; // new k for n
        let t = from_fword(n & !(0x007fffff >> k));
        n = ((n & 0x007fffff) | 0x00800000) >> (23 - k);
        if j < 0 {
            n = -n;
        }
        p_h -= t;
    }
    let t = trunc12(p_l + p_h);
    let u = t * LG2_H;
    let v = (p_l - (t - p_h)) * LG2 + t * LG2_L;
    z = u + v;
    let w = v - (z - u);
    let t = z * z;
    let t1 = z - t * (P1 + t * (P2 + t * (P3 + t * (P4 + t * P5))));
    let r = (z * t1) / (t1 - 2.0) - (w + z * w);
    z = 1.0 - (r - z);
    j = fword(z);
    j = j.wrapping_add(n << 23);
    if (j >> 23) <= 0 {
        z = scalbnf(z, n); // subnormal output
    } else {
        z = from_fword(j);
    }
    s * z
}

// sf_scalbn.c
fn scalbnf(mut x: f32, n: i32) -> f32 {
    const TWO25: f32 = 3.355443200e+07f64 as f32; // 0x4c000000
    const TWOM25: f32 = 2.9802322388e-08f64 as f32; // 0x33000000
    const HUGE: f32 = 1.0e+30f64 as f32;
    const TINY: f32 = 1.0e-30f64 as f32;

    let mut ix = fword(x);
    let hx = (ix & 0x7fffffff) as u32;
    let mut k = (hx >> 23) as i32; // extract exponent
    if hx == 0 {
        return x;
    }
    if hx >= 0x7f800000 {
        return x + x; // NaN or Inf
    }
    if hx < 0x00800000 {
        x *= TWO25;
        ix = fword(x);
        k = ((ix & 0x7f800000) >> 23) - 25;
        if n < -50000 {
            return TINY * x; // underflow
        }
    }
    if n > 50000 {
        // in case integer overflow in n+k
        return HUGE * copysignf(HUGE, x); // overflow
    }
    k += n;
    if k > 0xfe {
        return HUGE * copysignf(HUGE, x); // overflow
    }
    if k > 0 {
        // normal result
        return from_fword((ix & 0x807fffffu32 as i32) | (k << 23));
    }
    if k < -22 {
        return TINY * copysignf(TINY, x); // underflow
    }
    k += 25; // subnormal result
    x = from_fword((ix & 0x807fffffu32 as i32) | (k << 23));
    x * TWOM25
}

/****************************************************************
 * sin (s_sin.c, k_sin.c, k_cos.c, e_rem_pio2.c, k_rem_pio2.c)
 ****************************************************************/

pub fn sin(x: f64) -> f64 {
    let z = 0.0;

    // High word of x.
    let ix = high(x) & 0x7fffffff;

    // |x| ~< pi/4
    if ix <= 0x3fe921fb {
        return kernel_sin(x, z, 0);
    }
    // sin(Inf or NaN) is NaN
    if ix >= 0x7ff00000 {
        return x - x;
    }
    // argument reduction needed
    let (n, y) = ieee754_rem_pio2(x);
    match n & 3 {
        0 => kernel_sin(y[0], y[1], 1),
        1 => kernel_cos(y[0], y[1]),
        2 => -kernel_sin(y[0], y[1], 1),
        _ => -kernel_cos(y[0], y[1]),
    }
}

fn kernel_sin(x: f64, y: f64, iy: i32) -> f64 {
    const HALF: f64 = 5.00000000000000000000e-01; // 0x3FE00000, 0x00000000
    const S1: f64 = -1.66666666666666324348e-01; // 0xBFC55555, 0x55555549
    const S2: f64 = 8.33333333332248946124e-03; // 0x3F811111, 0x1110F8A6
    const S3: f64 = -1.98412698298579493134e-04; // 0xBF2A01A0, 0x19C161D5
    const S4: f64 = 2.75573137070700676789e-06; // 0x3EC71DE3, 0x57B1FE7D
    const S5: f64 = -2.50507602534068634195e-08; // 0xBE5AE5E6, 0x8A2B9CEB
    const S6: f64 = 1.58969099521155010221e-10; // 0x3DE5D93A, 0x5ACFD57C

    let ix = high(x) & 0x7fffffff; // high word of x
    if ix < 0x3e400000 && x as i32 == 0 {
        // |x| < 2**-27
        return x; // generate inexact
    }
    let z = x * x;
    let v = z * x;
    let r = S2 + z * (S3 + z * (S4 + z * (S5 + z * S6)));
    if iy == 0 {
        x + v * (S1 + z * r)
    } else {
        x - ((z * (HALF * y - v * r) - y) - v * S1)
    }
}

fn kernel_cos(x: f64, y: f64) -> f64 {
    const ONE: f64 = 1.00000000000000000000e+00; // 0x3FF00000, 0x00000000
    const C1: f64 = 4.16666666666666019037e-02; // 0x3FA55555, 0x5555554C
    const C2: f64 = -1.38888888888741095749e-03; // 0xBF56C16C, 0x16C15177
    const C3: f64 = 2.48015872894767294178e-05; // 0x3EFA01A0, 0x19CB1590
    const C4: f64 = -2.75573143513906633035e-07; // 0xBE927E4F, 0x809C52AD
    const C5: f64 = 2.08757232129817482790e-09; // 0x3E21EE9E, 0xBDB4B1C4
    const C6: f64 = -1.13596475577881948265e-11; // 0xBDA8FAE9, 0xBE8838D4

    let ix = high(x) & 0x7fffffff; // ix = |x|'s high word
    if ix < 0x3e400000 && x as i32 == 0 {
        // if x < 2**27
        return ONE; // generate inexact
    }
    let z = x * x;
    let r = z * (C1 + z * (C2 + z * (C3 + z * (C4 + z * (C5 + z * C6)))));
    if ix < 0x3FD33333 {
        // if |x| < 0.3
        ONE - (0.5 * z - (z * r - x * y))
    } else {
        let qx = if ix > 0x3fe90000 {
            0.28125 // x > 0.78125
        } else {
            words(ix - 0x00200000, 0) // x/4
        };
        let hz = 0.5 * z - qx;
        let a = ONE - qx;
        a - (hz - (z * r - x * y))
    }
}

// Table of constants for 2/pi, 396 Hex digits (476 decimal) of 2/pi
const TWO_OVER_PI: [i32; 66] = [
    0xA2F983, 0x6E4E44, 0x1529FC, 0x2757D1, 0xF534DD, 0xC0DB62, 0x95993C, 0x439041, 0xFE5163, 0xABDEBB, 0xC561B7,
    0x246E3A, 0x424DD2, 0xE00649, 0x2EEA09, 0xD1921C, 0xFE1DEB, 0x1CB129, 0xA73EE8, 0x8235F5, 0x2EBB44, 0x84E99C,
    0x7026B4, 0x5F7E41, 0x3991D6, 0x398353, 0x39F49C, 0x845F8B, 0xBDF928, 0x3B1FF8, 0x97FFDE, 0x05980F, 0xEF2F11,
    0x8B5A0A, 0x6D1F6D, 0x367ECF, 0x27CB09, 0xB74F46, 0x3F669E, 0x5FEA2D, 0x7527BA, 0xC7EBE5, 0xF17B3D, 0x0739F7,
    0x8A5292, 0xEA6BFB, 0x5FB11F, 0x8D5D08, 0x560330, 0x46FC7B, 0x6BABF0, 0xCFBC20, 0x9AF436, 0x1DA9E3, 0x91615E,
    0xE61B08, 0x659985, 0x5F14A0, 0x68408D, 0xFFD880, 0x4D7327, 0x310606, 0x1556CA, 0x73A8C9, 0x60E27B, 0xC08C6B,
];

const NPIO2_HW: [i32; 32] = [
    0x3FF921FB, 0x400921FB, 0x4012D97C, 0x401921FB, 0x401F6A7A, 0x4022D97C, 0x4025FDBB, 0x402921FB, 0x402C463A,
    0x402F6A7A, 0x4031475C, 0x4032D97C, 0x40346B9C, 0x4035FDBB, 0x40378FDB, 0x403921FB, 0x403AB41B, 0x403C463A,
    0x403DD85A, 0x403F6A7A, 0x40407E4C, 0x4041475C, 0x4042106C, 0x4042D97C, 0x4043A28C, 0x40446B9C, 0x404534AC,
    0x4045FDBB, 0x4046C6CB, 0x40478FDB, 0x404858EB, 0x404921FB,
];

const TWO24: f64 = 1.67772160000000000000e+07; // 0x41700000, 0x00000000
const TWON24: f64 = 5.96046447753906250000e-08; // 0x3E700000, 0x00000000

// return the remainder of x rem pi/2 in y[0]+y[1]
fn ieee754_rem_pio2(x: f64) -> (i32, [f64; 2]) {
    const HALF: f64 = 5.00000000000000000000e-01; // 0x3FE00000, 0x00000000
    const INVPIO2: f64 = 6.36619772367581382433e-01; // 0x3FE45F30, 0x6DC9C883
    const PIO2_1: f64 = 1.57079632673412561417e+00; // 0x3FF921FB, 0x54400000
    const PIO2_1T: f64 = 6.07710050650619224932e-11; // 0x3DD0B461, 0x1A626331
    const PIO2_2: f64 = 6.07710050630396597660e-11; // 0x3DD0B461, 0x1A600000
    const PIO2_2T: f64 = 2.02226624879595063154e-21; // 0x3BA3198A, 0x2E037073
    const PIO2_3: f64 = 2.02226624871116645580e-21; // 0x3BA3198A, 0x2E000000
    const PIO2_3T: f64 = 8.47842766036889956997e-32; // 0x397B839A, 0x252049C1

    let mut y = [0.0; 2];
    let hx = high(x); // high word of x
    let ix = hx & 0x7fffffff;
    if ix <= 0x3fe921fb {
        // |x| ~<= pi/4 , no need for reduction
        return (0, [x, 0.0]);
    }
    if ix < 0x4002d97c {
        // |x| < 3pi/4, special case with n=+-1
        if hx > 0 {
            let mut z = x - PIO2_1;
            if ix != 0x3ff921fb {
                // 33+53 bit pi is good enough
                y[0] = z - PIO2_1T;
                y[1] = (z - y[0]) - PIO2_1T;
            } else {
                // near pi/2, use 33+33+53 bit pi
                z -= PIO2_2;
                y[0] = z - PIO2_2T;
                y[1] = (z - y[0]) - PIO2_2T;
            }
            return (1, y);
        } else {
            // negative x
            let mut z = x + PIO2_1;
            if ix != 0x3ff921fb {
                // 33+53 bit pi is good enough
                y[0] = z + PIO2_1T;
                y[1] = (z - y[0]) + PIO2_1T;
            } else {
                // near pi/2, use 33+33+53 bit pi
                z += PIO2_2;
                y[0] = z + PIO2_2T;
                y[1] = (z - y[0]) + PIO2_2T;
            }
            return (-1, y);
        }
    }
    if ix <= 0x413921fb {
        // |x| ~<= 2^19*(pi/2), medium size
        let mut t = f64::from_bits(x.to_bits() & !(1 << 63)); // fabs(x)
        let n = (t * INVPIO2 + HALF) as i32;
        let fnn = n as f64;
        let mut r = t - fnn * PIO2_1;
        let mut w = fnn * PIO2_1T; // 1st round good to 85 bit
        if n < 32 && ix != NPIO2_HW[(n - 1) as usize] {
            y[0] = r - w; // quick check no cancellation
        } else {
            let j = ix >> 20;
            y[0] = r - w;
            let mut i = j - ((high(y[0]) >> 20) & 0x7ff);
            if i > 16 {
                // 2nd iteration needed, good to 118
                t = r;
                w = fnn * PIO2_2;
                r = t - w;
                w = fnn * PIO2_2T - ((t - r) - w);
                y[0] = r - w;
                i = j - ((high(y[0]) >> 20) & 0x7ff);
                if i > 49 {
                    // 3rd iteration need, 151 bits acc
                    t = r; // will cover all possible cases
                    w = fnn * PIO2_3;
                    r = t - w;
                    w = fnn * PIO2_3T - ((t - r) - w);
                    y[0] = r - w;
                }
            }
        }
        y[1] = (r - y[0]) - w;
        if hx < 0 {
            return (-n, [-y[0], -y[1]]);
        }
        return (n, y);
    }
    // all other (large) arguments
    if ix >= 0x7ff00000 {
        // x is inf or NaN
        return (0, [x - x, x - x]);
    }
    // set z = scalbn(|x|,ilogb(x)-23)
    let e0 = (ix >> 20) - 1046; // e0 = ilogb(z)-23;
    let mut z = words(ix - (e0 << 20), low(x));
    let mut tx = [0.0; 3];
    for t in tx.iter_mut().take(2) {
        *t = (z as i32) as f64;
        z = (z - *t) * TWO24;
    }
    tx[2] = z;
    let mut nx = 3;
    while tx[nx - 1] == 0.0 {
        nx -= 1; // skip zero term
    }
    let n = kernel_rem_pio2(&tx[..nx], &mut y, e0);
    if hx < 0 {
        return (-n, [-y[0], -y[1]]);
    }
    (n, y)
}

// __kernel_rem_pio2 at the precision __ieee754_rem_pio2 asks (prec 2:
// 53 bits, jk 4)
fn kernel_rem_pio2(x: &[f64], y: &mut [f64; 2], e0: i32) -> i32 {
    const PIO2: [f64; 8] = [
        1.57079625129699707031e+00, // 0x3FF921FB, 0x40000000
        7.54978941586159635335e-08, // 0x3E74442D, 0x00000000
        5.39030252995776476554e-15, // 0x3CF84698, 0x80000000
        3.28200341580791294123e-22, // 0x3B78CC51, 0x60000000
        1.27065575308067607349e-29, // 0x39F01B83, 0x80000000
        1.22933308981111328932e-36, // 0x387A2520, 0x40000000
        2.73370053816464559624e-44, // 0x36E38222, 0x80000000
        2.16741683877804819444e-51, // 0x3569F31D, 0x00000000
    ];
    let ipio2 = &TWO_OVER_PI;
    let mut iq = [0i32; 20];
    let (mut f, mut fq, mut q) = ([0.0f64; 20], [0.0f64; 20], [0.0f64; 20]);

    // initialize jk
    let jk: i32 = 4;
    let jp = jk;

    // determine jx,jv,q0, note that 3>q0
    let jx = x.len() as i32 - 1;
    let mut jv = (e0 - 3) / 24;
    if jv < 0 {
        jv = 0;
    }
    let mut q0 = e0 - 24 * (jv + 1);

    // set up f[0] to f[jx+jk] where f[jx+jk] = ipio2[jv+jk]
    let mut j = jv - jx;
    let m = jx + jk;
    for fi in f.iter_mut().take((m + 1) as usize) {
        *fi = if j < 0 { 0.0 } else { ipio2[j as usize] as f64 };
        j += 1;
    }

    // compute q[0],q[1],...q[jk]
    for i in 0..=jk {
        let mut fw = 0.0;
        for j in 0..=jx {
            fw += x[j as usize] * f[(jx + i - j) as usize];
        }
        q[i as usize] = fw;
    }

    let mut jz = jk;
    let (mut z, mut n, mut ih);
    loop {
        // distill q[] into iq[] reversingly
        let mut i = 0;
        let mut j = jz;
        z = q[jz as usize];
        while j > 0 {
            let fw = ((TWON24 * z) as i32) as f64;
            iq[i] = (z - TWO24 * fw) as i32;
            z = q[(j - 1) as usize] + fw;
            i += 1;
            j -= 1;
        }

        // compute n
        z = scalbn(z, q0); // actual value of z
        z -= 8.0 * floor(z * 0.125); // trim off integer >= 8
        n = z as i32;
        z -= n as f64;
        ih = 0;
        if q0 > 0 {
            // need iq[jz-1] to determine n
            let i = iq[(jz - 1) as usize] >> (24 - q0);
            n += i;
            iq[(jz - 1) as usize] -= i << (24 - q0);
            ih = iq[(jz - 1) as usize] >> (23 - q0);
        } else if q0 == 0 {
            ih = iq[(jz - 1) as usize] >> 23;
        } else if z >= 0.5 {
            ih = 2;
        }

        if ih > 0 {
            // q > 0.5
            n += 1;
            let mut carry = 0;
            for v in iq.iter_mut().take(jz as usize) {
                // compute 1-q
                let j = *v;
                if carry == 0 {
                    if j != 0 {
                        carry = 1;
                        *v = 0x1000000 - j;
                    }
                } else {
                    *v = 0xffffff - j;
                }
            }
            if q0 > 0 {
                // rare case: chance is 1 in 12
                match q0 {
                    1 => iq[(jz - 1) as usize] &= 0x7fffff,
                    2 => iq[(jz - 1) as usize] &= 0x3fffff,
                    _ => {}
                }
            }
            if ih == 2 {
                z = 1.0 - z;
                if carry != 0 {
                    z -= scalbn(1.0, q0);
                }
            }
        }

        // check if recomputation is needed
        if z == 0.0 {
            let mut j = 0;
            let mut i = jz - 1;
            while i >= jk {
                j |= iq[i as usize];
                i -= 1;
            }
            if j == 0 {
                // need recomputation
                let mut k = 1;
                while iq[(jk - k) as usize] == 0 {
                    k += 1; // k = no. of terms needed
                }
                for i in (jz + 1)..=(jz + k) {
                    // add q[jz+1] to q[jz+k]
                    f[(jx + i) as usize] = ipio2[(jv + i) as usize] as f64;
                    let mut fw = 0.0;
                    for j in 0..=jx {
                        fw += x[j as usize] * f[(jx + i - j) as usize];
                    }
                    q[i as usize] = fw;
                }
                jz += k;
                continue;
            }
        }
        break;
    }

    // chop off zero terms
    if z == 0.0 {
        jz -= 1;
        q0 -= 24;
        while iq[jz as usize] == 0 {
            jz -= 1;
            q0 -= 24;
        }
    } else {
        // break z into 24-bit if necessary
        z = scalbn(z, -q0);
        if z >= TWO24 {
            let fw = ((TWON24 * z) as i32) as f64;
            iq[jz as usize] = (z - TWO24 * fw) as i32;
            jz += 1;
            q0 += 24;
            iq[jz as usize] = fw as i32;
        } else {
            iq[jz as usize] = z as i32;
        }
    }

    // convert integer "bit" chunk to floating-point value
    let mut fw = scalbn(1.0, q0);
    let mut i = jz;
    while i >= 0 {
        q[i as usize] = fw * iq[i as usize] as f64;
        fw *= TWON24;
        i -= 1;
    }

    // compute PIo2[0,...,jp]*q[jz,...,0]
    let mut i = jz;
    while i >= 0 {
        let mut fw = 0.0;
        let mut k = 0;
        while k <= jp && k <= jz - i {
            fw += PIO2[k as usize] * q[(i + k) as usize];
            k += 1;
        }
        fq[(jz - i) as usize] = fw;
        i -= 1;
    }

    // compress fq[] into y[]
    let mut fw = 0.0;
    let mut i = jz;
    while i >= 0 {
        fw += fq[i as usize];
        i -= 1;
    }
    y[0] = if ih == 0 { fw } else { -fw };
    fw = fq[0] - fw;
    for i in 1..=jz {
        fw += fq[i as usize];
    }
    y[1] = if ih == 0 { fw } else { -fw };
    n & 7
}

// s_scalbn.c
fn scalbn(mut x: f64, n: i32) -> f64 {
    const TWO54: f64 = 1.80143985094819840000e+16; // 0x43500000, 0x00000000
    const TWOM54: f64 = 5.55111512312578270212e-17; // 0x3C900000, 0x00000000
    const HUGE: f64 = 1.0e+300;
    const TINY: f64 = 1.0e-300;

    let mut hx = high(x);
    let lx = low(x);
    let mut k = (hx & 0x7ff00000) >> 20; // extract exponent
    if k == 0 {
        // 0 or subnormal x
        if (lx | (hx & 0x7fffffff) as u32) == 0 {
            return x; // +-0
        }
        x *= TWO54;
        hx = high(x);
        k = ((hx & 0x7ff00000) >> 20) - 54;
        if n < -50000 {
            return TINY * x; // underflow
        }
    }
    if k == 0x7ff {
        return x + x; // NaN or Inf
    }
    if n > 50000 {
        // in case integer overflow in n+k
        return HUGE * copysign(HUGE, x); // overflow
    }
    k += n;
    if k > 0x7fe {
        return HUGE * copysign(HUGE, x); // overflow
    }
    if k > 0 {
        // normal result
        return words((hx & 0x800fffffu32 as i32) | (k << 20), low(x));
    }
    if k <= -54 {
        return TINY * copysign(TINY, x); // underflow
    }
    k += 54; // subnormal result
    x = words((hx & 0x800fffffu32 as i32) | (k << 20), low(x));
    x * TWOM54
}

// s_floor.c
fn floor(x: f64) -> f64 {
    const HUGE: f64 = 1.0e300;

    let mut i0 = high(x);
    let mut i1 = low(x);
    let j0 = ((i0 >> 20) & 0x7ff) - 0x3ff;
    if j0 < 20 {
        if j0 < 0 {
            // raise inexact if x != 0
            if HUGE + x > 0.0 {
                // return 0*sign(x) if |x|<1
                if i0 >= 0 {
                    i0 = 0;
                    i1 = 0;
                } else if ((i0 & 0x7fffffff) as u32 | i1) != 0 {
                    i0 = 0xbff00000u32 as i32;
                    i1 = 0;
                }
            }
        } else {
            let i = 0x000fffffu32 >> j0;
            if ((i0 as u32 & i) | i1) == 0 {
                return x; // x is integral
            }
            if HUGE + x > 0.0 {
                // raise inexact flag
                if i0 < 0 {
                    i0 += 0x00100000 >> j0;
                }
                i0 &= !(i as i32);
                i1 = 0;
            }
        }
    } else if j0 > 51 {
        if j0 == 0x400 {
            return x + x; // inf or NaN
        } else {
            return x; // x is integral
        }
    } else {
        let i = 0xffffffffu32 >> (j0 - 20);
        if (i1 & i) == 0 {
            return x; // x is integral
        }
        if HUGE + x > 0.0 {
            // raise inexact flag
            if i0 < 0 {
                if j0 == 20 {
                    i0 += 1;
                } else {
                    let j = i1.wrapping_add(1 << (52 - j0));
                    if j < i1 {
                        i0 += 1; // got a carry
                    }
                    i1 = j;
                }
            }
            i1 &= !i;
        }
    }
    words(i0, i1)
}
