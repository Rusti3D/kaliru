// klippy/extras/bus.py: MCU_SPI, MCU_I2C, MCU_bus_digital_out and their
// from_config helpers (Happy Hare's i2c / spi devices, accelerometers).
use super::mcu::{cq_from_py, cq_to_py, params_to_py};
use super::*;
use crate::extras::bus::{McuBusDigitalOut, McuI2c, McuSpi};
use crate::mcu::Mcu;
use crate::plugins::convert::{FromPy, ToPy};
use crate::{method, ro_attr};

/// bus data: a list of ints, bytes or bytearray
fn bytes_from_py(o: &PyRef) -> KResult<Vec<u8>> {
    let v: Vec<i64> = FromPy::from_py(o)?;
    Ok(v.into_iter().map(|x| x as u8).collect())
}

fn mcu(o: &PyRef) -> KResult<Rc<Mcu>> {
    objects::unwrap_as::<Mcu>(o)
}

fn u64_of(o: &PyRef) -> KResult<u64> {
    Ok(o.as_i64()? as u64)
}

/// sw_pins: None or a sequence of N pin names
fn sw_pins<const N: usize>(o: &PyRef) -> KResult<Option<[String; N]>> {
    if o.is_none() {
        return Ok(None);
    }
    let v: Vec<String> = FromPy::from_py(o)?;
    let n = v.len();
    v.try_into().map(Some).map_err(|_| KError::Py("ValueError".into(), format!("expected {} software pins, got {}", N, n)))
}

fn opt_cmd<T: Clone + 'static>(c: &RefCell<Option<T>>) -> KResult<PyRef> {
    match c.borrow().clone() {
        Some(c) => objects::wrap(Rc::new(c)),
        None => Ok(py_none()),
    }
}

pub fn register(r: &mut Registry) {
    objects::register_class::<McuSpi>("klippy.extras.bus", "MCU_SPI");
    objects::register_class::<McuI2c>("klippy.extras.bus", "MCU_I2C");
    objects::register_class::<McuBusDigitalOut>("klippy.extras.bus", "MCU_bus_digital_out");

    // resolve_bus_name(mcu, param, bus)
    r.method("klippy.extras.bus.resolve_bus_name", |a| {
        let bus: Option<String> = FromPy::from_py(a.arg(2))?;
        crate::extras::bus::resolve_bus_name(&*mcu(a.arg(0))?, &a.string(1)?, bus.as_deref()).to_py()
    });

    // ------------------------------------------------------------ MCU_SPI
    // MCU_SPI(mcu, bus, pin, mode, speed, sw_pins=None, cs_active_high=False)
    r.ctor("klippy.extras.bus.MCU_SPI.__init__", |a| {
        let bus: Option<String> = FromPy::from_py(a.arg(1))?;
        let pin: Option<String> = FromPy::from_py(a.arg(2))?;
        let s: Rc<dyn Any> =
            McuSpi::new(&mcu(a.arg(0))?, bus, pin.as_deref(), a.arg(3).as_i64()?, a.arg(4).as_i64()?, sw_pins::<3>(a.arg(5))?, a.bool(6)?);
        Ok(s)
    });
    r.method("klippy.extras.bus.MCU_SPI.setup_shutdown_msg", |a| {
        a.this::<McuSpi>()?.setup_shutdown_msg(&bytes_from_py(a.arg(0))?);
        Ok(py_none())
    });
    method!(r, "klippy.extras.bus.MCU_SPI", McuSpi, get_oid(s) => s.get_oid());
    method!(r, "klippy.extras.bus.MCU_SPI", McuSpi, get_mcu(s) => s.get_mcu());
    r.method("klippy.extras.bus.MCU_SPI.get_command_queue", |a| cq_to_py(a.this::<McuSpi>()?.get_command_queue()));
    method!(r, "klippy.extras.bus.MCU_SPI", McuSpi, build_config(s) => s.build_config());
    // spi_send(data, minclock=0, reqclock=0)
    r.method("klippy.extras.bus.MCU_SPI.spi_send", |a| {
        a.this::<McuSpi>()?.spi_send(&bytes_from_py(a.arg(0))?, u64_of(a.arg(1))?, u64_of(a.arg(2))?)?;
        Ok(py_none())
    });
    r.method("klippy.extras.bus.MCU_SPI.spi_send_wait_ack", |a| {
        a.this::<McuSpi>()?.spi_send_wait_ack(&bytes_from_py(a.arg(0))?, u64_of(a.arg(1))?, u64_of(a.arg(2))?)?;
        Ok(py_none())
    });
    r.method("klippy.extras.bus.MCU_SPI.spi_transfer", |a| {
        params_to_py(&a.this::<McuSpi>()?.spi_transfer(&bytes_from_py(a.arg(0))?, u64_of(a.arg(1))?, u64_of(a.arg(2))?)?)
    });
    r.method("klippy.extras.bus.MCU_SPI.spi_transfer_with_preface", |a| {
        let p = a.this::<McuSpi>()?.spi_transfer_with_preface(
            &bytes_from_py(a.arg(0))?,
            &bytes_from_py(a.arg(1))?,
            u64_of(a.arg(2))?,
            u64_of(a.arg(3))?,
        )?;
        params_to_py(&p)
    });
    ro_attr!(r, "klippy.extras.bus.MCU_SPI.mcu", McuSpi, |s| s.mcu.clone());
    ro_attr!(r, "klippy.extras.bus.MCU_SPI.bus", McuSpi, |s| s.bus.clone());
    ro_attr!(r, "klippy.extras.bus.MCU_SPI.speed", McuSpi, |s| s.speed);
    ro_attr!(r, "klippy.extras.bus.MCU_SPI.oid", McuSpi, |s| s.oid);
    ro_attr!(r, "klippy.extras.bus.MCU_SPI.config_fmt", McuSpi, |s| s.config_fmt.borrow().clone());
    ro_attr!(r, "klippy.extras.bus.MCU_SPI.config_fmt_ticks", McuSpi, |s| s.config_fmt_ticks.clone());
    r.attr("klippy.extras.bus.MCU_SPI.cmd_queue", |o| cq_to_py(objects::unwrap_as::<McuSpi>(o)?.cmd_queue.clone()), None);
    r.attr("klippy.extras.bus.MCU_SPI.spi_send_cmd", |o| opt_cmd(&objects::unwrap_as::<McuSpi>(o)?.spi_send_cmd), None);
    r.attr("klippy.extras.bus.MCU_SPI.spi_transfer_cmd", |o| opt_cmd(&objects::unwrap_as::<McuSpi>(o)?.spi_transfer_cmd), None);
    // MCU_SPI_from_config(config, mode, pin_option="cs_pin", default_speed=100000, share_type=None, cs_active_high=False)
    r.method("klippy.extras.bus.MCU_SPI_from_config", |a| {
        let config = super::configfile::config_from_py(a.arg(0))?;
        let share_type: Option<String> = FromPy::from_py(a.arg(4))?;
        crate::extras::bus::mcu_spi_from_config(&config, a.arg(1).as_i64()?, &a.string(2)?, a.arg(3).as_i64()?, share_type.as_deref(), a.bool(5)?)
            .to_py()
    });

    // ------------------------------------------------------------ MCU_I2C
    // MCU_I2C(mcu, bus, addr, speed, sw_pins=None)
    r.ctor("klippy.extras.bus.MCU_I2C.__init__", |a| {
        let bus: Option<String> = FromPy::from_py(a.arg(1))?;
        let i: Rc<dyn Any> = McuI2c::new(&mcu(a.arg(0))?, bus, a.arg(2).as_i64()?, a.arg(3).as_i64()?, sw_pins::<2>(a.arg(4))?);
        Ok(i)
    });
    // _handle_connect(): the writes queued before the connection
    r.method("klippy.extras.bus.MCU_I2C._handle_connect", |a| {
        let i = a.this::<McuI2c>()?;
        let pending = i.to_write.borrow().clone();
        for data in pending {
            i.i2c_write(&data, 0, 0)?;
        }
        Ok(py_none())
    });
    method!(r, "klippy.extras.bus.MCU_I2C", McuI2c, get_oid(i) => i.get_oid());
    method!(r, "klippy.extras.bus.MCU_I2C", McuI2c, get_mcu(i) => i.get_mcu());
    method!(r, "klippy.extras.bus.MCU_I2C", McuI2c, get_i2c_address(i) => i.get_i2c_address());
    r.method("klippy.extras.bus.MCU_I2C.get_command_queue", |a| cq_to_py(a.this::<McuI2c>()?.get_command_queue()));
    method!(r, "klippy.extras.bus.MCU_I2C", McuI2c, build_config(i) => i.build_config());
    r.method("klippy.extras.bus.MCU_I2C.i2c_write_noack", |a| {
        a.this::<McuI2c>()?.i2c_write_noack(&bytes_from_py(a.arg(0))?, u64_of(a.arg(1))?, u64_of(a.arg(2))?)?;
        Ok(py_none())
    });
    r.method("klippy.extras.bus.MCU_I2C.i2c_write", |a| {
        a.this::<McuI2c>()?.i2c_write(&bytes_from_py(a.arg(0))?, u64_of(a.arg(1))?, u64_of(a.arg(2))?)?;
        Ok(py_none())
    });
    // i2c_read(write, read_len, retry=True)
    r.method("klippy.extras.bus.MCU_I2C.i2c_read", |a| {
        params_to_py(&a.this::<McuI2c>()?.i2c_read(&bytes_from_py(a.arg(0))?, a.arg(1).as_i64()?, a.bool(2)?)?)
    });
    ro_attr!(r, "klippy.extras.bus.MCU_I2C.mcu", McuI2c, |i| i.mcu.clone());
    ro_attr!(r, "klippy.extras.bus.MCU_I2C.bus", McuI2c, |i| i.bus.clone());
    ro_attr!(r, "klippy.extras.bus.MCU_I2C.i2c_address", McuI2c, |i| i.i2c_address);
    ro_attr!(r, "klippy.extras.bus.MCU_I2C.oid", McuI2c, |i| i.oid);
    ro_attr!(r, "klippy.extras.bus.MCU_I2C.speed", McuI2c, |i| i.speed);
    ro_attr!(r, "klippy.extras.bus.MCU_I2C.config_fmt", McuI2c, |i| i.config_fmt.borrow().clone());
    ro_attr!(r, "klippy.extras.bus.MCU_I2C.config_fmt_ticks", McuI2c, |i| i.config_fmt_ticks.clone());
    r.attr("klippy.extras.bus.MCU_I2C.cmd_queue", |o| cq_to_py(objects::unwrap_as::<McuI2c>(o)?.cmd_queue.clone()), None);
    r.attr("klippy.extras.bus.MCU_I2C.i2c_write_cmd", |o| opt_cmd(&objects::unwrap_as::<McuI2c>(o)?.i2c_write_cmd), None);
    r.attr("klippy.extras.bus.MCU_I2C.i2c_read_cmd", |o| opt_cmd(&objects::unwrap_as::<McuI2c>(o)?.i2c_read_cmd), None);
    // _to_write: [bytes, ...]
    r.attr("klippy.extras.bus.MCU_I2C._to_write", |o| {
        let tw = objects::unwrap_as::<McuI2c>(o)?.to_write.borrow().clone();
        let bytes = lookup("builtins", "bytes")?;
        let items = tw.iter().map(|d| bytes.call(&[&py_list(d.iter().map(|x| py_int(*x as i64)).collect())])).collect::<KResult<Vec<_>>>()?;
        Ok(py_list(items))
    }, None);
    // MCU_I2C_from_config(config, default_addr=None, default_speed=100000)
    r.method("klippy.extras.bus.MCU_I2C_from_config", |a| {
        let config = super::configfile::config_from_py(a.arg(0))?;
        let addr: Option<i64> = FromPy::from_py(a.arg(1))?;
        crate::extras::bus::mcu_i2c_from_config(&config, addr, a.arg(2).as_i64()?).to_py()
    });

    // ------------------------------------------------ MCU_bus_digital_out
    // MCU_bus_digital_out(mcu, pin_desc, cmd_queue=None, value=0)
    r.ctor("klippy.extras.bus.MCU_bus_digital_out.__init__", |a| {
        let d: Rc<dyn Any> = McuBusDigitalOut::new(&mcu(a.arg(0))?, &a.string(1)?, cq_from_py(a.arg(2))?, a.arg(3).as_i64()?)?;
        Ok(d)
    });
    method!(r, "klippy.extras.bus.MCU_bus_digital_out", McuBusDigitalOut, get_oid(d) => d.get_oid());
    method!(r, "klippy.extras.bus.MCU_bus_digital_out", McuBusDigitalOut, get_mcu(d) => d.get_mcu());
    r.method("klippy.extras.bus.MCU_bus_digital_out.get_command_queue", |a| cq_to_py(a.this::<McuBusDigitalOut>()?.get_command_queue()));
    // build_config(): the update_digital_out command lookup
    r.method("klippy.extras.bus.MCU_bus_digital_out.build_config", |a| {
        let d = a.this::<McuBusDigitalOut>()?;
        let cmd = d.mcu.lookup_command("update_digital_out oid=%c value=%c", Some(d.cmd_queue.clone()))?;
        *d.update_pin_cmd.borrow_mut() = Some(cmd);
        Ok(py_none())
    });
    // update_digital_out(value, minclock=0, reqclock=0)
    r.method("klippy.extras.bus.MCU_bus_digital_out.update_digital_out", |a| {
        a.this::<McuBusDigitalOut>()?.update_digital_out(a.arg(0).as_i64()?, u64_of(a.arg(1))?, u64_of(a.arg(2))?)?;
        Ok(py_none())
    });
    ro_attr!(r, "klippy.extras.bus.MCU_bus_digital_out.mcu", McuBusDigitalOut, |d| d.mcu.clone());
    ro_attr!(r, "klippy.extras.bus.MCU_bus_digital_out.oid", McuBusDigitalOut, |d| d.oid);
    r.attr("klippy.extras.bus.MCU_bus_digital_out.cmd_queue", |o| cq_to_py(objects::unwrap_as::<McuBusDigitalOut>(o)?.cmd_queue.clone()), None);
    r.attr("klippy.extras.bus.MCU_bus_digital_out.update_pin_cmd", |o| {
        opt_cmd(&objects::unwrap_as::<McuBusDigitalOut>(o)?.update_pin_cmd)
    }, None);
}
