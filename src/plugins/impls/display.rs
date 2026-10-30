// klippy/extras/display: the lcd chips' framebuffers and io (Kalico's
// bytearrays as snapshots: python sees their contents, writes to them do
// not reach the display), DisplayTemplate
use super::*;
use crate::extras::display::hd44780::TextFramebuffers;
use crate::plugins::convert::ToPy;
use crate::ro_attr;

fn bytearray(b: &[u8]) -> KResult<PyRef> {
    lookup("builtins", "bytearray")?.call(&[&lookup("builtins", "bytes")?.call(&[&b.to_vec().to_py()?])?])
}

/// [(new data, data on the display, command / id)]
fn all_framebuffers(fbs: &[(Vec<u8>, Vec<u8>, i64)]) -> KResult<PyRef> {
    let items: KResult<Vec<PyRef>> = fbs.iter().map(|(n, o, id)| Ok(py_tuple(vec![bytearray(n)?, bytearray(o)?, py_int(*id)]))).collect();
    Ok(py_list(items?))
}

// hd44780 / hd44780_spi / aip31068_spi: two text framebuffers and the glyph one
fn text_framebuffers(fb: &TextFramebuffers) -> KResult<PyRef> {
    let fbs = fb.fbs.borrow();
    Ok(py_list(vec![bytearray(&fbs[0].0)?, bytearray(&fbs[1].0)?]))
}

fn glyph_framebuffer(fb: &TextFramebuffers) -> KResult<PyRef> {
    bytearray(&fb.fbs.borrow()[2].0)
}

/// {name: (slot, rows)}
fn icons(fb: &TextFramebuffers) -> KResult<PyRef> {
    let d = lookup("builtins", "dict")?.call(&[])?;
    for (k, (slot, rows)) in fb.icons.borrow().iter() {
        d.call_method("__setitem__", &[&py_str(k), &py_tuple(vec![py_int(*slot), rows.clone().to_py()?])])?;
    }
    Ok(d)
}

macro_rules! text_chip {
    ($r:expr, $class:literal, $ty:ty) => {
        $r.attr(concat!($class, ".all_framebuffers"), |o| all_framebuffers(&objects::unwrap_as::<$ty>(o)?.fb.fbs.borrow()), None);
        $r.attr(concat!($class, ".text_framebuffers"), |o| text_framebuffers(&objects::unwrap_as::<$ty>(o)?.fb), None);
        $r.attr(concat!($class, ".glyph_framebuffer"), |o| glyph_framebuffer(&objects::unwrap_as::<$ty>(o)?.fb), None);
        $r.attr(concat!($class, ".icons"), |o| icons(&objects::unwrap_as::<$ty>(o)?.fb), None);
        $r.attr(concat!($class, ".line_length"), |o| Ok(py_int(objects::unwrap_as::<$ty>(o)?.fb.line_length)), None);
    };
}

fn cmd_to_py(c: &crate::mcu::CommandWrapper) -> KResult<PyRef> {
    Rc::new(c.clone()).to_py()
}

pub fn register(r: &mut Registry) {
    use crate::extras::display as d;
    text_chip!(r, "klippy.extras.display.hd44780.HD44780", d::hd44780::HD44780);
    text_chip!(r, "klippy.extras.display.hd44780_spi.hd44780_spi", d::hd44780_spi::HD44780Spi);
    text_chip!(r, "klippy.extras.display.aip31068_spi.aip31068_spi", d::aip31068_spi::Aip31068Spi);
    // send(cmds, is_data=False)
    r.method("klippy.extras.display.hd44780.HD44780.send", |a| {
        let cmds: Vec<u8> = crate::plugins::convert::FromPy::from_py(&lookup("builtins", "list")?.call(&[a.arg(0)])?)?;
        a.this::<d::hd44780::HD44780>()?.send(&cmds, a.arg(1).is_true()?, 0, 0)?;
        Ok(py_none())
    });
    r.attr("klippy.extras.display.hd44780.HD44780.send_cmds_cmd", |o| match objects::unwrap_as::<d::hd44780::HD44780>(o)?.cmds.borrow().as_ref() {
        Some((c, _)) => cmd_to_py(c),
        None => Ok(py_none()),
    }, None);
    r.attr("klippy.extras.display.hd44780.HD44780.send_data_cmd", |o| match objects::unwrap_as::<d::hd44780::HD44780>(o)?.cmds.borrow().as_ref() {
        Some((_, c)) => cmd_to_py(c),
        None => Ok(py_none()),
    }, None);

    // st7920: text (64), glyph (128) and 32 graphics framebuffers
    {
        use d::st7920::{DisplayBase, Io};
        r.attr("klippy.extras.display.st7920.DisplayBase.all_framebuffers", |o| all_framebuffers(&objects::unwrap_as::<DisplayBase>(o)?.fbs.borrow()), None);
        r.attr("klippy.extras.display.st7920.DisplayBase.text_framebuffer", |o| bytearray(&objects::unwrap_as::<DisplayBase>(o)?.fbs.borrow()[0].0), None);
        r.attr("klippy.extras.display.st7920.DisplayBase.glyph_framebuffer", |o| bytearray(&objects::unwrap_as::<DisplayBase>(o)?.fbs.borrow()[1].0), None);
        r.attr("klippy.extras.display.st7920.DisplayBase.graphics_framebuffers", |o| {
            let items: KResult<Vec<PyRef>> = objects::unwrap_as::<DisplayBase>(o)?.fbs.borrow()[2..].iter().map(|f| bytearray(&f.0)).collect();
            Ok(py_list(items?))
        }, None);
        fn native<R>(o: &PyRef, f: impl FnOnce(&Rc<crate::mcu::Mcu>, i64, &[String; 3], &std::cell::RefCell<Option<(crate::mcu::CommandWrapper, crate::mcu::CommandWrapper)>>) -> KResult<R>) -> KResult<R> {
            match &objects::unwrap_as::<DisplayBase>(o)?.io {
                Io::Native { mcu, oid, pins, cmds } => f(mcu, *oid, pins, cmds),
                _ => Err(KError::Py("AttributeError".into(), "not an ST7920".into())),
            }
        }
        r.attr("klippy.extras.display.st7920.ST7920.oid", |o| native(o, |_, oid, _, _| Ok(py_int(oid))), None);
        r.attr("klippy.extras.display.st7920.ST7920.pins", |o| native(o, |_, _, pins, _| pins.to_vec().to_py()), None);
        r.attr("klippy.extras.display.st7920.ST7920.send_cmds_cmd", |o| native(o, |_, _, _, c| match c.borrow().as_ref() {
            Some((c, _)) => cmd_to_py(c),
            None => Ok(py_none()),
        }), None);
        r.attr("klippy.extras.display.st7920.ST7920.send_data_cmd", |o| native(o, |_, _, _, c| match c.borrow().as_ref() {
            Some((_, c)) => cmd_to_py(c),
            None => Ok(py_none()),
        }), None);
        r.method("klippy.extras.display.st7920.ST7920.build_config", |a| {
            d::st7920::build_config(&*a.this::<DisplayBase>()?)?;
            Ok(py_none())
        });
        fn emulated<R>(o: &PyRef, f: impl FnOnce(&Rc<crate::extras::bus::McuSpi>, &Rc<d::st7920::EnableHelper>, bool) -> KResult<R>) -> KResult<R> {
            match &objects::unwrap_as::<DisplayBase>(o)?.io {
                Io::Emulated { spi, en_helper, en_set } => f(spi, en_helper, en_set.get()),
                _ => Err(KError::Py("AttributeError".into(), "not an EmulatedST7920".into())),
            }
        }
        r.attr("klippy.extras.display.st7920.EmulatedST7920.spi", |o| emulated(o, |s, _, _| s.clone().to_py()), None);
        r.attr("klippy.extras.display.st7920.EmulatedST7920.en_helper", |o| emulated(o, |_, h, _| h.clone().to_py()), None);
        r.attr("klippy.extras.display.st7920.EmulatedST7920.en_set", |o| emulated(o, |_, _, e| Ok(py_bool(e))), None);
    }

    // uc1701 / ssd1306 / sh1106: 8 vram pages
    {
        use d::uc1701::{Chip, DisplayBase};
        r.attr("klippy.extras.display.uc1701.DisplayBase.all_framebuffers", |o| {
            let fbs: Vec<(Vec<u8>, Vec<u8>, i64)> =
                objects::unwrap_as::<DisplayBase>(o)?.vram.borrow().iter().enumerate().map(|(i, (n, o))| (n.clone(), o.clone(), i as i64)).collect();
            all_framebuffers(&fbs)
        }, None);
        ro_attr!(r, "klippy.extras.display.uc1701.DisplayBase.columns", DisplayBase, |s| s.vram.borrow()[0].0.len() as i64);
        // io.send(cmds, is_data=False)
        r.attr("klippy.extras.display.uc1701.DisplayBase.send", |o| {
            let s = objects::unwrap_as::<DisplayBase>(o)?;
            crate::plugins::callable::py_function(Rc::new(move |a: &[PyRef]| {
                let cmds: Vec<u8> = crate::plugins::convert::FromPy::from_py(&lookup("builtins", "list")?.call(&[&a[0]])?)?;
                let is_data = match a.get(1) {
                    Some(v) => v.is_true()?,
                    None => false,
                };
                s.send(&cmds, is_data)?;
                Ok(py_none())
            }))
        }, None);
        fn chip<R>(o: &PyRef, f: impl FnOnce(&Chip) -> R) -> KResult<R> {
            Ok(f(&objects::unwrap_as::<DisplayBase>(o)?.chip))
        }
        r.attr("klippy.extras.display.uc1701.UC1701.contrast", |o| chip(o, |c| match c {
            Chip::UC1701 { contrast } | Chip::SSD1306 { contrast, .. } => py_int(*contrast),
        }), None);
        r.attr("klippy.extras.display.uc1701.SSD1306.contrast", |o| chip(o, |c| match c {
            Chip::UC1701 { contrast } | Chip::SSD1306 { contrast, .. } => py_int(*contrast),
        }), None);
        r.attr("klippy.extras.display.uc1701.SSD1306.vcomh", |o| chip(o, |c| match c {
            Chip::SSD1306 { vcomh, .. } => py_int(*vcomh),
            _ => py_none(),
        }), None);
        r.attr("klippy.extras.display.uc1701.SSD1306.invert", |o| chip(o, |c| match c {
            Chip::SSD1306 { invert, .. } => py_bool(*invert),
            _ => py_none(),
        }), None);
        ro_attr!(r, "klippy.extras.display.uc1701.UC1701.reset", DisplayBase, |s| s.reset.clone());
        ro_attr!(r, "klippy.extras.display.uc1701.SSD1306.reset", DisplayBase, |s| s.reset.clone());
    }

    // ---- display templates
    {
        use crate::extras::display_templates::DisplayTemplate;
        r.attr("klippy.extras.display.display.DisplayTemplate.params", |o| {
            let t = objects::unwrap_as::<DisplayTemplate>(o)?;
            let dct = lookup("builtins", "dict")?.call(&[])?;
            for (k, v) in &t.params {
                dct.call_method("__setitem__", &[&py_str(k), &crate::plugins::values::value_to_py(v)?])?;
            }
            Ok(dct)
        }, None);
        // render(context, **kwargs)
        r.method("klippy.extras.display.display.DisplayTemplate.render", |a| {
            let context = super::extras_misc::template_context(Some(a.arg(0).clone()))?.unwrap_or_default();
            let kwargs = super::extras_misc::template_context(Some(a.arg(1).clone()))?.unwrap_or_default();
            Ok(py_str(&a.this::<DisplayTemplate>()?.render(&context, &kwargs)?))
        });
    }
}
