// klippy/extras/heaters.py: PrinterHeaters, Heater. Plugins put their own
// heaters into PrinterHeaters.heaters and its name lists (AFC_ACE's dryer):
// kaliru keeps them (HeaterSlot::Plugin) and calls them by name.
use super::*;
use crate::extras::heaters::{Heater, HeaterSlot, PrinterHeaters};
use crate::method;
use crate::plugins::convert::ToPy;
use crate::plugins::native::{DictField, ListField};
use crate::plugins::py_to_pobj;

fn ph(o: &PyRef) -> KResult<Rc<PrinterHeaters>> {
    objects::unwrap_as::<PrinterHeaters>(o)
}

pub fn register(r: &mut Registry) {
    method!(r, "klippy.extras.heaters.PrinterHeaters", PrinterHeaters, lookup_heater(p, name: String) => p.lookup_heater_obj(&name));
    method!(r, "klippy.extras.heaters.PrinterHeaters", PrinterHeaters, get_all_heaters(p) => p.get_all_heaters());
    method!(r, "klippy.extras.heaters.PrinterHeaters", PrinterHeaters, turn_off_all_heaters(p) => p.turn_off_all_heaters());
    method!(r, "klippy.extras.heaters.PrinterHeaters", PrinterHeaters, set_temperature(p, heater: PObj, temp: f64, wait: bool) => {
        match crate::printer::downcast_rc::<Heater>(heater) {
            Some(h) => p.set_temperature(&h, temp, wait),
            None => Err(KError::Py("NotImplementedError".into(), "kaliru: set_temperature of a python plugin's heater".into())),
        }
    });
    let names = ListField {
        get: |o, field| {
            let p = ph(o)?;
            let which = field.rsplit('.').next().unwrap_or("");
            let list = p.name_list(which).ok_or_else(|| KError::Other(format!("kaliru: no list {}", field)))?;
            let out = list.borrow().iter().map(|s| py_str(s)).collect();
            Ok(out)
        },
        set: |o, field, items| {
            let p = ph(o)?;
            let which = field.rsplit('.').next().unwrap_or("");
            let list = p.name_list(which).ok_or_else(|| KError::Other(format!("kaliru: no list {}", field)))?;
            *list.borrow_mut() = items.iter().map(|i| i.as_string()).collect::<KResult<_>>()?;
            Ok(())
        },
    };
    r.list("klippy.extras.heaters.PrinterHeaters.available_heaters", names);
    r.list("klippy.extras.heaters.PrinterHeaters.available_sensors", names);
    r.list("klippy.extras.heaters.PrinterHeaters.available_monitors", names);
    r.dict(
        "klippy.extras.heaters.PrinterHeaters.heaters",
        DictField {
            get: |o, _f, key| match ph(o)?.heaters.borrow().get(&key.as_string()?) {
                Some(slot) => slot.as_pobj().to_py(),
                None => Err(KError::Py("KeyError".into(), key.repr()?)),
            },
            set: |o, _f, key, value| {
                let pobj = py_to_pobj(value);
                let slot = match crate::printer::downcast_rc::<Heater>(pobj.clone()) {
                    Some(h) => HeaterSlot::Kaliru(h),
                    None => HeaterSlot::Plugin(pobj),
                };
                ph(o)?.heaters.borrow_mut().insert(key.as_string()?, slot);
                Ok(())
            },
            del: |o, _f, key| match ph(o)?.heaters.borrow_mut().shift_remove(&key.as_string()?) {
                Some(_) => Ok(()),
                None => Err(KError::Py("KeyError".into(), key.repr()?)),
            },
            keys: |o, _f| Ok(ph(o)?.heaters.borrow().keys().map(|k| py_str(k)).collect()),
        },
    );
}
