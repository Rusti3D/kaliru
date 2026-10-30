// klippy/mcu.py: MCUConnectHelper / MCUConfigHelper / MCUStatsHelper /
// MCURestartHelper. Kaliru's Mcu keeps their state itself (mcu.rs conn /
// cfg / stats_h / restart); python gets one helper object per mcu and kind
use super::*;
use crate::mcu::Mcu;
use crate::plugins::convert::ToPy;
use std::collections::HashMap;

#[derive(Clone, Copy, PartialEq, Eq, Hash)]
pub enum Kind {
    Connect,
    Config,
    Stats,
    Restart,
}

pub struct McuHelper {
    pub mcu: Rc<Mcu>,
    pub kind: Kind,
}

static HELPERS: Gil<RefCell<HashMap<(usize, Kind), Rc<McuHelper>>>> = Gil::new(|| RefCell::new(HashMap::new()));

fn helper(mcu: &Rc<Mcu>, kind: Kind) -> KResult<PyRef> {
    let key = (Rc::as_ptr(mcu) as usize, kind);
    let h = HELPERS.with(|h| h.borrow_mut().entry(key).or_insert_with(|| Rc::new(McuHelper { mcu: mcu.clone(), kind })).clone());
    h.to_py()
}

fn mcu_of(a: &crate::plugins::native::Args) -> KResult<Rc<Mcu>> {
    Ok(a.this::<McuHelper>()?.mcu.clone())
}

fn hm(o: &PyRef) -> KResult<Rc<Mcu>> {
    Ok(objects::unwrap_as::<McuHelper>(o)?.mcu.clone())
}

pub fn register(r: &mut Registry) {
    objects::register_class_fn::<McuHelper>(|a| match a.downcast_ref::<McuHelper>().map(|h| h.kind) {
        Some(Kind::Connect) => ("klippy.mcu", "MCUConnectHelper"),
        Some(Kind::Config) => ("klippy.mcu", "MCUConfigHelper"),
        Some(Kind::Stats) => ("klippy.mcu", "MCUStatsHelper"),
        _ => ("klippy.mcu", "MCURestartHelper"),
    });
    r.attr("klippy.mcu.MCU._conn_helper", |o| helper(&objects::unwrap_as::<Mcu>(o)?, Kind::Connect), None);
    r.attr("klippy.mcu.MCU._config_helper", |o| helper(&objects::unwrap_as::<Mcu>(o)?, Kind::Config), None);
    r.attr("klippy.mcu.MCU._stats_helper", |o| helper(&objects::unwrap_as::<Mcu>(o)?, Kind::Stats), None);

    // ---- MCUConnectHelper
    r.method("klippy.mcu.MCUConnectHelper.get_mcu", |a| mcu_of(a)?.to_py());
    r.method("klippy.mcu.MCUConnectHelper.get_serial", |a| mcu_of(a)?.get_serial().to_py());
    r.method("klippy.mcu.MCUConnectHelper.get_clocksync", |a| mcu_of(a)?.clocksync.clone().to_py());
    r.method("klippy.mcu.MCUConnectHelper.get_restart_helper", |a| helper(&mcu_of(a)?, Kind::Restart));
    r.method("klippy.mcu.MCUConnectHelper.get_serialport", |a| {
        let m = mcu_of(a)?;
        let port = m.conn.serialport.borrow().clone();
        Ok(py_tuple(vec![py_str(&port), py_int(m.conn.baud.get())]))
    });
    r.method("klippy.mcu.MCUConnectHelper.log_info", |a| Ok(py_str(&mcu_of(a)?.conn_log_info())));
    r.method("klippy.mcu.MCUConnectHelper.check_serial_exists", |a| Ok(py_bool(mcu_of(a)?.conn_check_serial_exists()?)));
    r.method("klippy.mcu.MCUConnectHelper.check_timeout", |a| {
        mcu_of(a)?.conn_check_timeout(a.f64(0)?)?;
        Ok(py_none())
    });
    r.method("klippy.mcu.MCUConnectHelper.force_local_shutdown", |a| {
        mcu_of(a)?.conn_force_local_shutdown()?;
        Ok(py_none())
    });
    r.method("klippy.mcu.MCUConnectHelper.disconnect", |a| {
        mcu_of(a)?.conn_disconnect();
        Ok(py_none())
    });
    r.method("klippy.mcu.MCUConnectHelper.is_shutdown", |a| Ok(py_bool(mcu_of(a)?.conn.is_shutdown.get())));
    r.method("klippy.mcu.MCUConnectHelper.get_shutdown_msg", |a| Ok(py_str(&mcu_of(a)?.conn.shutdown_msg.borrow())));
    r.attr("klippy.mcu.MCUConnectHelper._mcu", |o| hm(o)?.to_py(), None);
    r.attr("klippy.mcu.MCUConnectHelper._serial", |o| hm(o)?.get_serial().to_py(), None);
    r.attr("klippy.mcu.MCUConnectHelper._clocksync", |o| hm(o)?.clocksync.clone().to_py(), None);
    r.attr("klippy.mcu.MCUConnectHelper._name", |o| Ok(py_str(&hm(o)?.name)), None);
    r.attr("klippy.mcu.MCUConnectHelper._serialport", |o| Ok(py_str(&hm(o)?.conn.serialport.borrow())), None);
    r.attr("klippy.mcu.MCUConnectHelper._baud", |o| Ok(py_int(hm(o)?.conn.baud.get())), None);
    r.attr("klippy.mcu.MCUConnectHelper._canbus_iface", |o| hm(o)?.conn.canbus_iface.borrow().clone().to_py(), None);
    r.attr("klippy.mcu.MCUConnectHelper._is_shutdown", |o| Ok(py_bool(hm(o)?.conn.is_shutdown.get())), None);
    r.attr("klippy.mcu.MCUConnectHelper._is_timeout", |o| Ok(py_bool(hm(o)?.conn.is_timeout.get())), None);
    r.attr("klippy.mcu.MCUConnectHelper._shutdown_msg", |o| Ok(py_str(&hm(o)?.conn.shutdown_msg.borrow())), None);
    r.attr("klippy.mcu.MCUConnectHelper._restart_helper", |o| helper(&hm(o)?, Kind::Restart), None);

    // ---- MCURestartHelper
    r.method("klippy.mcu.MCURestartHelper.check_restart_on_crc_mismatch", |a| {
        mcu_of(a)?.restart_check_restart_on_crc_mismatch()?;
        Ok(py_none())
    });
    r.method("klippy.mcu.MCURestartHelper.check_restart_on_send_config", |a| {
        mcu_of(a)?.restart_check_restart_on_send_config()?;
        Ok(py_none())
    });
    r.method("klippy.mcu.MCURestartHelper.check_restart_on_attach", |a| {
        mcu_of(a)?.restart_check_restart_on_attach()?;
        Ok(py_none())
    });
    r.method("klippy.mcu.MCURestartHelper.lookup_attach_uart_rts", |a| Ok(py_bool(mcu_of(a)?.restart_lookup_attach_uart_rts())));
    r.method("klippy.mcu.MCURestartHelper.recon_handle_shutdown", |a| Ok(py_bool(mcu_of(a)?.restart_recon_handle_shutdown()?)));
    r.attr("klippy.mcu.MCURestartHelper._mcu", |o| hm(o)?.to_py(), None);
    r.attr("klippy.mcu.MCURestartHelper._name", |o| Ok(py_str(&hm(o)?.name)), None);
    r.attr("klippy.mcu.MCURestartHelper._restart_method", |o| hm(o)?.restart.restart_method.borrow().clone().to_py(), None);
    r.attr("klippy.mcu.MCURestartHelper._is_mcu_bridge", |o| Ok(py_bool(hm(o)?.restart.is_mcu_bridge.get())), None);
    r.attr("klippy.mcu.MCURestartHelper._conn_helper", |o| helper(&hm(o)?, Kind::Connect), None);

    // ---- MCUStatsHelper: set_status_info(key, value)
    r.method("klippy.mcu.MCUStatsHelper.set_status_info", |a| {
        mcu_of(a)?.stats_set_status_info(&a.string(0)?, to_pyval(a.arg(1))?);
        Ok(py_none())
    });
    r.attr("klippy.mcu.MCUStatsHelper._mcu", |o| hm(o)?.to_py(), None);
    r.attr("klippy.mcu.MCUStatsHelper._mcu_freq", |o| Ok(py_float(hm(o)?.stats_h.mcu_freq.get())), None);
    r.attr("klippy.mcu.MCUStatsHelper._get_status_info", |o| Ok(from_pyval(&PyVal::Dict(hm(o)?.stats_h.get_status_info.borrow().clone()))), None);

    // ---- MCUConfigHelper
    r.method("klippy.mcu.MCUConfigHelper.create_oid", |a| Ok(py_int(mcu_of(a)?.create_oid())));
    r.method("klippy.mcu.MCUConfigHelper.register_config_callback", |a| {
        let cb = a.arg(0).clone();
        mcu_of(a)?.register_config_callback(Rc::new(move || {
            cb.call(&[])?;
            Ok(())
        }));
        Ok(py_none())
    });
    // add_config_cmd(cmd, is_init=False, on_restart=False)
    r.method("klippy.mcu.MCUConfigHelper.add_config_cmd", |a| {
        mcu_of(a)?.add_config_cmd(&a.string(0)?, a.bool(1)?, a.bool(2)?);
        Ok(py_none())
    });
    r.method("klippy.mcu.MCUConfigHelper.get_query_slot", |a| Ok(py_int(mcu_of(a)?.get_query_slot(a.arg(0).as_i64()?))));
    r.method("klippy.mcu.MCUConfigHelper.seconds_to_clock", |a| Ok(py_int(mcu_of(a)?.seconds_to_clock(a.f64(0)?))));
    r.method("klippy.mcu.MCUConfigHelper.request_move_queue_slot", |a| {
        mcu_of(a)?.request_move_queue_slot();
        Ok(py_none())
    });
    r.method("klippy.mcu.MCUConfigHelper.dump_debug", |a| Ok(py_str(&mcu_of(a)?.dump_debug())));
    r.method("klippy.mcu.MCUConfigHelper.reset_to_initial_state", |a| {
        mcu_of(a)?.cfg_reset_to_initial_state();
        Ok(py_none())
    });
    r.method("klippy.mcu.MCUConfigHelper.connect_config", |a| {
        mcu_of(a)?.cfg_connect_config()?;
        Ok(py_none())
    });
    r.method("klippy.mcu.MCUConfigHelper.setup_pin", |a| {
        let pin_type = a.string(0)?;
        let params = super::pins::pin_params_from_py(a.arg(1))?;
        let pin = mcu_of(a)?.cfg_setup_pin(&pin_type, &params).map_err(KError::Pins)?;
        super::pins::pin_to_py(pin, &pin_type)
    });
    r.attr("klippy.mcu.MCUConfigHelper._mcu", |o| hm(o)?.to_py(), None);
    r.attr("klippy.mcu.MCUConfigHelper._oid_count", |o| Ok(py_int(hm(o)?.cfg.oid_count.get())), None);
    r.attr("klippy.mcu.MCUConfigHelper._mcu_freq", |o| Ok(py_float(hm(o)?.cfg.mcu_freq.get())), None);
    r.attr("klippy.mcu.MCUConfigHelper._config_cmds", |o| hm(o)?.cfg.config_cmds.borrow().clone().to_py(), None);
    r.attr("klippy.mcu.MCUConfigHelper._init_cmds", |o| hm(o)?.cfg.init_cmds.borrow().clone().to_py(), None);
    r.attr("klippy.mcu.MCUConfigHelper._restart_cmds", |o| hm(o)?.cfg.restart_cmds.borrow().clone().to_py(), None);
    r.attr("klippy.mcu.MCUConfigHelper._reserved_move_slots", |o| Ok(py_int(hm(o)?.cfg.reserved_move_slots.get())), None);
    r.attr("klippy.mcu.MCUConfigHelper._config_crc", |o| Ok(py_int(hm(o)?.cfg.config_crc.get() as i64)), None);
    r.attr("klippy.mcu.MCUConfigHelper._conn_helper", |o| helper(&hm(o)?, Kind::Connect), None);

    // the rest of the helpers' state
    fn cmd_or_none(c: Option<crate::mcu::CommandWrapper>) -> KResult<PyRef> {
        match c {
            Some(c) => Rc::new(c).to_py(),
            None => Ok(py_none()),
        }
    }
    r.attr("klippy.mcu.MCUConfigHelper._cached_init_state", |o| Ok(py_bool(hm(o)?.cfg.cached_init_state.get())), None);
    r.attr("klippy.mcu.MCUConfigHelper._oid_count_post_inits", |o| Ok(py_int(hm(o)?.cfg.oid_count_post_inits.get())), None);
    r.attr("klippy.mcu.MCUConfigHelper._config_cmds_post_inits", |o| hm(o)?.cfg.config_cmds_post_inits.borrow().clone().to_py(), None);
    r.attr("klippy.mcu.MCUConfigHelper._init_cmds_post_inits", |o| hm(o)?.cfg.init_cmds_post_inits.borrow().clone().to_py(), None);
    r.attr("klippy.mcu.MCUConfigHelper._restart_cmds_post_inits", |o| hm(o)?.cfg.restart_cmds_post_inits.borrow().clone().to_py(), None);
    r.attr("klippy.mcu.MCUConnectHelper._emergency_stop_cmd", |o| cmd_or_none(hm(o)?.conn.emergency_stop_cmd.borrow().clone()), None);
    r.attr("klippy.mcu.MCURestartHelper._reset_cmd", |o| cmd_or_none(hm(o)?.restart.reset_cmd.borrow().clone()), None);
    r.attr("klippy.mcu.MCURestartHelper._config_reset_cmd", |o| cmd_or_none(hm(o)?.restart.config_reset_cmd.borrow().clone()), None);
    r.attr("klippy.mcu.MCUStatsHelper._mcu_tick_avg", |o| Ok(py_float(hm(o)?.stats_h.mcu_tick_avg.get())), None);
    r.attr("klippy.mcu.MCUStatsHelper._mcu_tick_awake", |o| Ok(py_float(hm(o)?.stats_h.mcu_tick_awake.get())), None);
    r.attr("klippy.mcu.MCUStatsHelper._mcu_tick_stddev", |o| Ok(py_float(hm(o)?.stats_h.mcu_tick_stddev.get())), None);
    r.attr("klippy.mcu.MCUStatsHelper._stats_sumsq_base", |o| Ok(py_float(hm(o)?.stats_h.stats_sumsq_base.get())), None);
}
