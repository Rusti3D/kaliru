// Exclude moves toward and inside objects (port of klippy/extras/exclude_object.py)
//
// Copyright (C) 2019  Eric Callahan <arksine.code@gmail.com>
// Copyright (C) 2021  Troy Jacobson <troy.d.jacobson@gmail.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::ConfigWrapper;
use crate::error::{KError, KResult};
use crate::extras::gcode_move::{GCodeMove, MoveTransform, MoveTransformRef};
use crate::extras::tuning_tower::TuningTower;
use crate::gcode::{GCodeCommand, GCodeDispatch};
use crate::logging;
use crate::printer::{downcast_rc, PObj, Printer, PrinterObject};
use crate::pyval::{PyDict, PyVal};
use crate::toolhead::ToolHead;
use crate::IndexMap;
use std::cell::{Cell, RefCell};
use std::rc::{Rc, Weak};

const CMD_EXCLUDE_OBJECT_START_HELP: &str = "Marks the beginning the current object as labeled";
const CMD_EXCLUDE_OBJECT_END_HELP: &str = "Marks the end the current object";
const CMD_EXCLUDE_OBJECT_HELP: &str = "Cancel moves inside a specified objects";
const CMD_EXCLUDE_OBJECT_DEFINE_HELP: &str = "Provides a summary of an object";

/// python max(a, b)
fn py_max(a: f64, b: f64) -> f64 {
    if b > a {
        b
    } else {
        a
    }
}

#[derive(Default)]
struct State {
    next_transform: Option<MoveTransformRef>,
    last_position_extruded: Vec<f64>,
    last_position_excluded: Vec<f64>,
    objects: Vec<PyDict>,
    excluded_objects: Vec<String>,
    current_object: Option<String>,
    in_excluded_region: bool,
    extrusion_offsets: IndexMap<String, Vec<f64>>,
    max_position_extruded: f64,
    max_position_excluded: f64,
    extruder_adj: f64,
    initial_extrusion_moves: i64,
    last_position: Vec<f64>,
}

pub struct ExcludeObject {
    me: Weak<ExcludeObject>,
    printer: Rc<Printer>,
    gcode: Rc<GCodeDispatch>,
    gcode_move: Rc<GCodeMove>,
    /// False when enable_exclude_object is off: python then never sets the
    /// state attributes (get_status raises AttributeError)
    enabled: bool,
    toolhead: RefCell<Option<Rc<ToolHead>>>,
    st: RefCell<State>,
    was_excluded_at_start: Cell<bool>,
}

type CmdFn = fn(&ExcludeObject, &GCodeCommand) -> KResult<()>;

impl ExcludeObject {
    fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
        let gcode_move = printer
            .load_object(config, "gcode_move", true)?
            .and_then(downcast_rc::<GCodeMove>)
            .ok_or_else(|| KError::Other("gcode_move has an unexpected type".into()))?;
        let enabled = config.getboolean("enable_exclude_object", true)?;
        let eo = Rc::new_cyclic(|me| ExcludeObject {
            me: me.clone(),
            printer: printer.clone(),
            gcode: gcode.clone(),
            gcode_move,
            enabled,
            toolhead: RefCell::new(None),
            st: RefCell::new(State::default()),
            was_excluded_at_start: Cell::new(false),
        });
        if !enabled {
            return Ok(eo);
        }

        let e = eo.clone();
        crate::events::KlippyConnect::on(&printer, move || {
            *e.toolhead.borrow_mut() = Some(e.printer.lookup::<ToolHead>("toolhead")?);
            Ok(())
        });
        let e = eo.clone();
        crate::events::VirtualSdcardResetFile::on(&printer, move || e.reset_file());
        {
            let mut st = eo.st.borrow_mut();
            st.last_position_extruded = vec![0.0; 4];
            st.last_position_excluded = vec![0.0; 4];
        }
        eo.reset_state();
        let cmds: [(&str, CmdFn, &str); 4] = [
            ("EXCLUDE_OBJECT_START", ExcludeObject::cmd_exclude_object_start, CMD_EXCLUDE_OBJECT_START_HELP),
            ("EXCLUDE_OBJECT_END", ExcludeObject::cmd_exclude_object_end, CMD_EXCLUDE_OBJECT_END_HELP),
            ("EXCLUDE_OBJECT", ExcludeObject::cmd_exclude_object, CMD_EXCLUDE_OBJECT_HELP),
            ("EXCLUDE_OBJECT_DEFINE", ExcludeObject::cmd_exclude_object_define, CMD_EXCLUDE_OBJECT_DEFINE_HELP),
        ];
        for (name, f, desc) in cmds {
            let e = eo.clone();
            gcode.register_command(name, Some(Rc::new(move |g| f(&e, g))), false, Some(desc))?;
        }
        Ok(eo)
    }

    fn register_transform(&self) -> KResult<()> {
        if self.st.borrow().next_transform.is_none() {
            let tuning_tower = self.printer.lookup::<TuningTower>("tuning_tower")?;
            if tuning_tower.is_active() {
                logging::info("The ExcludeObject move transform is not being loaded due to Tuning tower being Active");
                return Ok(());
            }

            let me: MoveTransformRef = self.me.upgrade().expect("exclude_object");
            let next_transform = self.gcode_move.set_move_transform(me, true)?;
            {
                let mut st = self.st.borrow_mut();
                st.next_transform = next_transform;
                st.extrusion_offsets.clear();
                st.max_position_extruded = 0.0;
                st.max_position_excluded = 0.0;
                st.extruder_adj = 0.0;
                st.initial_extrusion_moves = 5;
                st.last_position = vec![0.0; 4];
            }

            self.get_position();
            let mut st = self.st.borrow_mut();
            let lp = st.last_position.clone();
            st.last_position_extruded = lp.clone();
            st.last_position_excluded = lp;
        }
        Ok(())
    }

    fn unregister_transform(&self) -> KResult<()> {
        let next_transform = self.st.borrow().next_transform.clone();
        if let Some(next_transform) = next_transform {
            let tuning_tower = self.printer.lookup::<TuningTower>("tuning_tower")?;
            if tuning_tower.is_active() {
                logging::error(
                    "The Exclude Object move transform was not unregistered because it is not at the head of the transform chain.",
                );
                return Ok(());
            }

            self.gcode_move.set_move_transform(next_transform, true)?;
            self.st.borrow_mut().next_transform = None;
            self.gcode_move.reset_last_position()?;
        }
        Ok(())
    }

    fn reset_state(&self) {
        let mut st = self.st.borrow_mut();
        st.objects.clear();
        st.excluded_objects.clear();
        st.current_object = None;
        st.in_excluded_region = false;
    }

    fn reset_file(&self) -> KResult<()> {
        self.reset_state();
        self.unregister_transform()
    }

    fn extruder_name(&self) -> String {
        match self.toolhead.borrow().as_ref() {
            Some(th) => th.get_extruder().get_name(),
            None => String::new(),
        }
    }

    fn get_extrusion_offsets(st: &mut State, ename: &str, num_coord: usize) -> Vec<f64> {
        let offset = st.extrusion_offsets.entry(ename.to_string()).or_insert_with(|| vec![0.0; num_coord]);
        if offset.len() < num_coord {
            offset.resize(num_coord, 0.0);
        }
        offset.clone()
    }

    fn set_extrusion_offsets(st: &mut State, ename: &str, offset: Vec<f64>) {
        st.extrusion_offsets.insert(ename.to_string(), offset);
    }

    fn normal_move(&self, newpos: &[f64], speed: f64) -> KResult<()> {
        let ename = self.extruder_name();
        let (tx_pos, next_transform) = {
            let mut st = self.st.borrow_mut();
            let mut offset = Self::get_extrusion_offsets(&mut st, &ename, newpos.len());

            if st.initial_extrusion_moves > 0 && st.last_position[3] != newpos[3] {
                // Since the transform is not loaded until there is a request to
                // exclude an object, the transform needs to track a few extrusions
                // to get the state of the extruder
                st.initial_extrusion_moves -= 1;
            }

            st.last_position = newpos.to_vec();
            st.last_position_extruded = st.last_position.clone();
            st.max_position_extruded = py_max(st.max_position_extruded, newpos[3]);

            // These next few conditionals handle the moves immediately after leaving
            // and excluded object.  The toolhead is at the end of the last printed
            // object and the gcode is at the end of the last excluded object.
            //
            // Ideally, there will be Z and E moves right away to adjust any offsets
            // before moving away from the last position.  Any remaining corrections
            // will be made on the firs XY move.
            let lpe = st.last_position_excluded.clone();
            if (offset[0] != 0.0 || offset[1] != 0.0) && (newpos[0] != lpe[0] || newpos[1] != lpe[1]) {
                for (i, o) in offset.iter_mut().enumerate() {
                    if i != 3 {
                        *o = 0.0;
                    }
                }
                offset[3] += st.extruder_adj;
                st.extruder_adj = 0.0;
            }

            if offset[2] != 0.0 && newpos[2] != lpe[2] {
                offset[2] = 0.0;
            }

            if st.extruder_adj != 0.0 && newpos[3] != lpe[3] {
                offset[3] += st.extruder_adj;
                st.extruder_adj = 0.0;
            }

            let tx_pos: Vec<f64> = newpos.iter().zip(&offset).map(|(n, o)| n - o).collect();
            Self::set_extrusion_offsets(&mut st, &ename, offset);
            (tx_pos, st.next_transform.clone())
        };
        match next_transform {
            Some(nt) => nt.move_to(&tx_pos, speed),
            None => Err(KError::Other("'NoneType' object has no attribute 'move'".into())),
        }
    }

    fn ignore_move(&self, newpos: &[f64]) {
        let ename = self.extruder_name();
        let mut st = self.st.borrow_mut();
        let mut offset = Self::get_extrusion_offsets(&mut st, &ename, newpos.len());
        for (i, o) in offset.iter_mut().enumerate() {
            if i != 3 {
                *o = newpos[i] - st.last_position_extruded[i];
            }
        }
        offset[3] = offset[3] + newpos[3] - st.last_position[3];
        Self::set_extrusion_offsets(&mut st, &ename, offset);
        st.last_position = newpos.to_vec();
        st.last_position_excluded = st.last_position.clone();
        st.max_position_excluded = py_max(st.max_position_excluded, newpos[3]);
    }

    fn move_into_excluded_region(&self, newpos: &[f64]) {
        self.st.borrow_mut().in_excluded_region = true;
        self.ignore_move(newpos);
    }

    fn move_from_excluded_region(&self, newpos: &[f64], speed: f64) -> KResult<()> {
        {
            let mut st = self.st.borrow_mut();
            st.in_excluded_region = false;

            // This adjustment value is used to compensate for any retraction
            // differences between the last object printed and excluded one.
            st.extruder_adj = st.max_position_excluded
                - st.last_position_excluded[3]
                - (st.max_position_extruded - st.last_position_extruded[3]);
        }
        self.normal_move(newpos, speed)
    }

    pub fn test_in_excluded_region(&self) -> bool {
        // Inside cancelled object
        let st = self.st.borrow();
        let in_excluded = match &st.current_object {
            Some(c) => st.excluded_objects.contains(c),
            None => false,
        };
        in_excluded && st.initial_extrusion_moves == 0
    }

    fn cmd_exclude_object_start(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let name = gcmd.get("NAME")?.to_uppercase();
        let known = self.st.borrow().objects.iter().any(|obj| obj.get("name").map(|n| n.str()) == Some(name.clone()));
        if !known {
            let mut def = IndexMap::default();
            def.insert("name".into(), PyVal::Str(name.clone()));
            self.add_object_definition(def);
        }
        self.st.borrow_mut().current_object = Some(name);
        self.was_excluded_at_start.set(self.test_in_excluded_region());
        Ok(())
    }

    fn cmd_exclude_object_end(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let (current, has_transform) = {
            let st = self.st.borrow();
            (st.current_object.clone(), st.next_transform.is_some())
        };
        if current.is_none() && has_transform {
            gcmd.respond_info("EXCLUDE_OBJECT_END called, but no object is currently active", true);
            return Ok(());
        }
        if let Some(name) = gcmd.get_opt("NAME") {
            if Some(name.to_uppercase()) != current {
                gcmd.respond_info(
                    &format!(
                        "EXCLUDE_OBJECT_END NAME={} does not match the current object NAME={}",
                        name.to_uppercase(),
                        current.unwrap_or_else(|| "None".into())
                    ),
                    true,
                );
            }
        }
        self.st.borrow_mut().current_object = None;
        Ok(())
    }

    fn cmd_exclude_object(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let reset = gcmd.get_opt("RESET").filter(|s| !s.is_empty());
        let current = gcmd.get_opt("CURRENT").filter(|s| !s.is_empty());
        let name = gcmd.get_or("NAME", "").to_uppercase();

        if reset.is_some() {
            if !name.is_empty() {
                self.unexclude_object(&name);
            } else {
                self.st.borrow_mut().excluded_objects.clear();
            }
        } else if !name.is_empty() {
            if !self.st.borrow().excluded_objects.contains(&name.to_uppercase()) {
                self.exclude_object(&name.to_uppercase())?;
            }
        } else if current.is_some() {
            let current_object = self.st.borrow().current_object.clone().filter(|c| !c.is_empty());
            match current_object {
                None => return Err(KError::command("There is no current object to cancel")),
                Some(c) => self.exclude_object(&c)?,
            }
        } else {
            self.list_excluded_objects(gcmd);
        }
        Ok(())
    }

    fn cmd_exclude_object_define(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let reset = gcmd.get_opt("RESET").filter(|s| !s.is_empty());
        let name = gcmd.get_or("NAME", "").to_uppercase();

        if reset.is_some() {
            self.reset_file()?;
        } else if !name.is_empty() {
            let mut parameters = gcmd.get_command_parameters();
            parameters.shift_remove("NAME");
            let center = parameters.shift_remove("CENTER");
            let polygon = parameters.shift_remove("POLYGON");

            let mut obj = IndexMap::default();
            obj.insert("name".into(), PyVal::Str(name.to_uppercase()));
            for (k, v) in parameters {
                obj.insert(k.into(), PyVal::Str(v));
            }

            if let Some(center) = center {
                obj.insert("center".into(), PyVal::json_loads(&format!("[{}]", center)).map_err(KError::Other)?);
            }

            if let Some(polygon) = polygon {
                obj.insert("polygon".into(), PyVal::json_loads(&polygon).map_err(KError::Other)?);
            }

            self.add_object_definition(obj);
        } else {
            self.list_objects(gcmd);
        }
        Ok(())
    }

    fn add_object_definition(&self, definition: PyDict) {
        let mut st = self.st.borrow_mut();
        st.objects.push(definition);
        // sorted(..., key=lambda o: o["name"]): a stable sort
        st.objects.sort_by(|a, b| {
            let name = |o: &PyDict| o.get("name").map(|n| n.str()).unwrap_or_default();
            name(a).cmp(&name(b))
        });
    }

    fn exclude_object(&self, name: &str) -> KResult<()> {
        self.register_transform()?;
        self.gcode.respond_info(&format!("Excluding object {}", name.to_uppercase()), true);
        let mut st = self.st.borrow_mut();
        if !st.excluded_objects.iter().any(|o| o == name) {
            st.excluded_objects.push(name.to_string());
            st.excluded_objects.sort();
        }
        Ok(())
    }

    fn unexclude_object(&self, name: &str) {
        self.gcode.respond_info(&format!("Unexcluding object {}", name.to_uppercase()), true);
        let mut st = self.st.borrow_mut();
        if let Some(pos) = st.excluded_objects.iter().position(|o| o == name) {
            st.excluded_objects.remove(pos);
            st.excluded_objects.sort();
        }
    }

    fn list_objects(&self, gcmd: &GCodeCommand) {
        let st = self.st.borrow();
        let object_list = if gcmd.get_opt("JSON").is_some() {
            PyVal::List(st.objects.iter().cloned().map(PyVal::Dict).collect()).json()
        } else {
            st.objects.iter().map(|obj| obj.get("name").map(|n| n.str()).unwrap_or_default()).collect::<Vec<_>>().join(" ")
        };
        gcmd.respond_info(&format!("Known objects: {}", object_list), true);
    }

    fn list_excluded_objects(&self, gcmd: &GCodeCommand) {
        let object_list = self.st.borrow().excluded_objects.join(" ");
        gcmd.respond_info(&format!("Excluded objects: {}", object_list), true);
    }
}

impl MoveTransform for ExcludeObject {
    fn get_position(&self) -> Vec<f64> {
        let next_transform = self.st.borrow().next_transform.clone();
        let pos = match next_transform {
            Some(nt) => nt.get_position(),
            None => return vec![],
        };
        let ename = self.extruder_name();
        let mut st = self.st.borrow_mut();
        let offset = Self::get_extrusion_offsets(&mut st, &ename, pos.len());
        if st.last_position.len() < pos.len() {
            st.last_position.resize(pos.len(), 0.0);
        }
        for i in 0..pos.len() {
            st.last_position[i] = pos[i] + offset[i];
        }
        st.last_position.clone()
    }

    fn move_to(&self, newpos: &[f64], speed: f64) -> KResult<()> {
        let move_in_excluded_region = self.test_in_excluded_region();
        let in_excluded_region = self.st.borrow().in_excluded_region;

        if move_in_excluded_region {
            if in_excluded_region {
                self.ignore_move(newpos);
            } else {
                self.move_into_excluded_region(newpos);
            }
            Ok(())
        } else if in_excluded_region {
            self.move_from_excluded_region(newpos, speed)
        } else {
            self.normal_move(newpos, speed)
        }
    }
}

impl PrinterObject for ExcludeObject {
    fn has_status(&self) -> bool {
        true
    }
    fn get_status(&self, _eventtime: f64) -> Option<PyVal> {
        let st = self.st.borrow();
        let mut status = IndexMap::default();
        status.insert("objects".into(), PyVal::List(st.objects.iter().cloned().map(PyVal::Dict).collect()));
        status.insert(
            "excluded_objects".into(),
            PyVal::List(st.excluded_objects.iter().cloned().map(PyVal::Str).collect()),
        );
        status.insert("current_object".into(), st.current_object.clone().map(PyVal::Str).unwrap_or(PyVal::None));
        Some(PyVal::Dict(status))
    }
    fn try_get_status(&self, eventtime: f64) -> KResult<Option<PyVal>> {
        if !self.enabled {
            return Err(KError::Other("'ExcludeObject' object has no attribute 'objects'".into()));
        }
        Ok(self.get_status(eventtime))
    }
    fn teardown(&self) {
        self.st.borrow_mut().next_transform = None;
        *self.toolhead.borrow_mut() = None;
    }
}

pub fn load_config(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(ExcludeObject::new(config)?)
}
