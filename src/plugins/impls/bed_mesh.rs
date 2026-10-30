// klippy/extras/bed_mesh.py: BedMesh, BedMeshCalibrate, ZMesh, MoveSplitter,
// ProfileManager and the module helpers. Probes that build their own mesh
// (beacon, cartographer: ZMesh(bmc.mesh_config.copy(), name).build_mesh(m);
// bed_mesh.set_mesh(mesh); bed_mesh.save_profile(name)) go through here.
use super::*;
use crate::extras::bed_mesh::{BedMesh, BedMeshCalibrate, MeshParams, MoveSplitter, ProfileManager, ZMesh, ZrefMode};
use crate::extras::probe::FinalizeResult;
use crate::gcode::GCodeCommand;
use crate::plugins::convert::{FromPy, ToPy};
use crate::plugins::native::DictField;
use crate::{cell_attr, function, method, ro_attr, vec_attr};

fn cfg(o: &PyRef) -> KResult<Rc<crate::configfile::ConfigWrapper>> {
    super::configfile::config_from_py(o)
}

fn gcmd(o: &PyRef) -> KResult<Rc<GCodeCommand>> {
    super::gcode::gcmd_from_py(o)
}

fn bm(o: &PyRef) -> KResult<Rc<BedMesh>> {
    objects::unwrap_as::<BedMesh>(o)
}

fn bmc(o: &PyRef) -> KResult<Rc<BedMeshCalibrate>> {
    objects::unwrap_as::<BedMeshCalibrate>(o)
}

fn zm(o: &PyRef) -> KResult<Rc<ZMesh>> {
    objects::unwrap_as::<ZMesh>(o)
}

fn ms(o: &PyRef) -> KResult<Rc<MoveSplitter>> {
    objects::unwrap_as::<MoveSplitter>(o)
}

fn params_to_py(p: &MeshParams) -> PyRef {
    from_pyval(&PyVal::Dict(p.clone()))
}

fn params_from_py(o: &PyRef) -> KResult<MeshParams> {
    match to_pyval(o)? {
        PyVal::Dict(d) => Ok(d),
        _ => Err(KError::Py("TypeError".into(), "kaliru: mesh params must be a dict".into())),
    }
}

fn opt_mesh_from_py(o: &PyRef) -> KResult<Option<Rc<ZMesh>>> {
    if o.is_none() {
        return Ok(None);
    }
    zm(o).map(Some)
}

/// build_mesh's error: Kalico's BedMeshError, or the TypeError of an unknown algorithm
fn mesh_error(msg: String) -> KError {
    if msg.starts_with("'NoneType' object is not callable") {
        return KError::Py("TypeError".into(), msg);
    }
    KError::Py("klippy.extras.bed_mesh.BedMeshError".into(), msg)
}

/// a print_func(msg) argument (errors reported: kaliru's printers return nothing)
fn print_func(f: &PyRef) -> impl Fn(&str) + '_ {
    move |m: &str| {
        if let Err(e) = f.call(&[&py_str(m)]) {
            report_unhandled(&e);
        }
    }
}

/// an error(msg) argument (config.error / gcmd.error): kaliru's error of it
fn error_fn(err: &PyRef) -> impl Fn(String) -> KError + '_ {
    move |m: String| match err.call(&[&py_str(&m)]) {
        Ok(exc) => crate::plugins::py::exception_to_kerror(exc),
        Err(e) => e,
    }
}

fn pair_to_py(p: (f64, f64)) -> PyRef {
    py_tuple(vec![py_float(p.0), py_float(p.1)])
}

fn finalize_to_py(r: FinalizeResult) -> PyRef {
    match r {
        FinalizeResult::Num(n) => py_float(n),
        FinalizeResult::Retry => py_str("retry"),
        FinalizeResult::Done => py_none(),
    }
}

fn zref_to_i64(m: ZrefMode) -> i64 {
    match m {
        ZrefMode::Disabled => 0,
        ZrefMode::InMesh => 1,
        ZrefMode::Probe => 2,
    }
}

pub fn register(r: &mut Registry) {
    // ------------------------------------------------------ module helpers
    function!(r, "klippy.extras.bed_mesh", isclose(a: f64, b: f64, rel_tol: f64, abs_tol: f64) =>
        crate::extras::bed_mesh::isclose(a, b, rel_tol, abs_tol));
    function!(r, "klippy.extras.bed_mesh", within(coord: (f64, f64), min_c: (f64, f64), max_c: (f64, f64), tol: f64) =>
        crate::extras::bed_mesh::within(coord, min_c, max_c, tol));
    function!(r, "klippy.extras.bed_mesh", constrain(val: f64, min_val: f64, max_val: f64) => crate::extras::bed_mesh::constrain(val, min_val, max_val));
    function!(r, "klippy.extras.bed_mesh", lerp(t: f64, v0: f64, v1: f64) => crate::extras::bed_mesh::lerp(t, v0, v1));
    function!(r, "klippy.extras.bed_mesh", parse_config_pair(config: PyRef, option: String, default: i64, minval: Option<i64>, maxval: Option<i64>) =>
        crate::extras::bed_mesh::parse_config_pair(&*cfg(&config)?, &option, default, minval, maxval));
    function!(r, "klippy.extras.bed_mesh", parse_gcmd_pair(g: PyRef, name: String, minval: Option<i64>, maxval: Option<i64>) =>
        crate::extras::bed_mesh::parse_gcmd_pair(&*gcmd(&g)?, &name, minval, maxval));
    function!(r, "klippy.extras.bed_mesh", parse_gcmd_coord(g: PyRef, name: String) =>
        crate::extras::bed_mesh::parse_gcmd_coord(&*gcmd(&g)?, &name));
    r.method("klippy.extras.bed_mesh.load_config", |a| crate::extras::bed_mesh::load_config(&*cfg(a.arg(0))?)?.to_py());

    // ------------------------------------------------------------ BedMesh
    r.ctor("klippy.extras.bed_mesh.BedMesh.__init__", |a| {
        let b: Rc<dyn Any> = BedMesh::new(&*cfg(a.arg(0))?)?;
        Ok(b)
    });
    method!(r, "klippy.extras.bed_mesh.BedMesh", BedMesh, handle_connect(b) => b.handle_connect());
    method!(r, "klippy.extras.bed_mesh.BedMesh", BedMesh, set_mesh(b, mesh: PyRef) => b.set_mesh(opt_mesh_from_py(&mesh)?));
    method!(r, "klippy.extras.bed_mesh.BedMesh", BedMesh, get_z_factor(b, z_pos: f64) => b.get_z_factor(z_pos));
    method!(r, "klippy.extras.bed_mesh.BedMesh", BedMesh, get_position(b) => b.bm_get_position());
    r.method("klippy.extras.bed_mesh.BedMesh.move", |a| {
        let newpos: Vec<f64> = FromPy::from_py(a.arg(0))?;
        a.this::<BedMesh>()?.bm_move(&newpos, a.f64(1)?)?;
        Ok(py_none())
    });
    // get_status(eventtime=None)
    r.method("klippy.extras.bed_mesh.BedMesh.get_status", |a| Ok(from_pyval(&a.this::<BedMesh>()?.status.borrow())));
    method!(r, "klippy.extras.bed_mesh.BedMesh", BedMesh, update_status(b) => b.update_status());
    method!(r, "klippy.extras.bed_mesh.BedMesh", BedMesh, get_mesh(b) => b.get_mesh());
    method!(r, "klippy.extras.bed_mesh.BedMesh", BedMesh, cmd_BED_MESH_OUTPUT(b, g: PyRef) => b.cmd_bed_mesh_output(&*gcmd(&g)?));
    method!(r, "klippy.extras.bed_mesh.BedMesh", BedMesh, cmd_BED_MESH_MAP(b, g: PyRef) => b.cmd_bed_mesh_map(&*gcmd(&g)?));
    method!(r, "klippy.extras.bed_mesh.BedMesh", BedMesh, cmd_BED_MESH_CLEAR(b, g: PyRef) => b.cmd_bed_mesh_clear(&*gcmd(&g)?));
    method!(r, "klippy.extras.bed_mesh.BedMesh", BedMesh, cmd_BED_MESH_OFFSET(b, g: PyRef) => b.cmd_bed_mesh_offset(&*gcmd(&g)?));
    method!(r, "klippy.extras.bed_mesh.BedMesh", BedMesh, cmd_BED_MESH_CHECK(b, g: PyRef) => b.cmd_bed_mesh_check(&*gcmd(&g)?));
    ro_attr!(r, "klippy.extras.bed_mesh.BedMesh.printer", BedMesh, |b| b.printer.clone());
    ro_attr!(r, "klippy.extras.bed_mesh.BedMesh.gcode", BedMesh, |b| b.printer.lookup::<crate::gcode::GCodeDispatch>("gcode"));
    ro_attr!(r, "klippy.extras.bed_mesh.BedMesh.bmc", BedMesh, |b| b.bmc.borrow().clone());
    ro_attr!(r, "klippy.extras.bed_mesh.BedMesh.pmgr", BedMesh, |b| b.pmgr.borrow().clone());
    ro_attr!(r, "klippy.extras.bed_mesh.BedMesh.splitter", BedMesh, |b| b.splitter.borrow().clone());
    ro_attr!(r, "klippy.extras.bed_mesh.BedMesh.toolhead", BedMesh, |b| b.toolhead.borrow().clone());
    r.attr("klippy.extras.bed_mesh.BedMesh.z_mesh", |o| bm(o)?.z_mesh.borrow().clone().to_py(), Some(|o, v| {
        *bm(o)?.z_mesh.borrow_mut() = opt_mesh_from_py(v)?;
        Ok(())
    }));
    cell_attr!(r, "klippy.extras.bed_mesh.BedMesh.horizontal_move_z", BedMesh, horizontal_move_z);
    cell_attr!(r, "klippy.extras.bed_mesh.BedMesh.fade_start", BedMesh, fade_start);
    cell_attr!(r, "klippy.extras.bed_mesh.BedMesh.fade_end", BedMesh, fade_end);
    cell_attr!(r, "klippy.extras.bed_mesh.BedMesh.fade_dist", BedMesh, fade_dist);
    cell_attr!(r, "klippy.extras.bed_mesh.BedMesh.log_fade_complete", BedMesh, log_fade_complete);
    cell_attr!(r, "klippy.extras.bed_mesh.BedMesh.base_fade_target", BedMesh, base_fade_target);
    cell_attr!(r, "klippy.extras.bed_mesh.BedMesh.fade_target", BedMesh, fade_target);
    cell_attr!(r, "klippy.extras.bed_mesh.BedMesh.tool_offset", BedMesh, tool_offset);
    vec_attr!(r, "klippy.extras.bed_mesh.BedMesh.last_position", BedMesh, last_position);
    r.attr("klippy.extras.bed_mesh.BedMesh.default_mesh_name", |o| bm(o)?.default_mesh_name.borrow().clone().to_py(), Some(|o, v| {
        *bm(o)?.default_mesh_name.borrow_mut() = FromPy::from_py(v)?;
        Ok(())
    }));
    r.attr("klippy.extras.bed_mesh.BedMesh.status", |o| Ok(from_pyval(&bm(o)?.status.borrow())), Some(|o, v| {
        *bm(o)?.status.borrow_mut() = to_pyval(v)?;
        Ok(())
    }));
    // self.save_profile = self.pmgr.save_profile
    r.attr("klippy.extras.bed_mesh.BedMesh.save_profile", |o| {
        let pmgr = bm(o)?.pmgr.borrow().clone().ok_or_else(|| KError::Py("AttributeError".into(), "save_profile".into()))?;
        objects::wrap(pmgr)?.getattr("save_profile")
    }, None);

    // --------------------------------------------------- BedMeshCalibrate
    r.ctor("klippy.extras.bed_mesh.BedMeshCalibrate.__init__", |a| {
        let b: Rc<dyn Any> = BedMeshCalibrate::new(&*cfg(a.arg(0))?, &bm(a.arg(1))?)?;
        Ok(b)
    });
    // _generate_points(error, probe_method="automatic")
    r.method("klippy.extras.bed_mesh.BedMeshCalibrate._generate_points", |a| {
        let err = a.arg(0).clone();
        a.this::<BedMeshCalibrate>()?.generate_points(&error_fn(&err), &a.string(1)?)?;
        Ok(py_none())
    });
    // print_generated_points(print_func, truncate=False)
    r.method("klippy.extras.bed_mesh.BedMeshCalibrate.print_generated_points", |a| {
        let f = a.arg(0).clone();
        a.this::<BedMeshCalibrate>()?.print_generated_points(&print_func(&f), a.bool(1)?)?;
        Ok(py_none())
    });
    r.method("klippy.extras.bed_mesh.BedMeshCalibrate._verify_algorithm", |a| {
        let b = a.this::<BedMeshCalibrate>()?;
        let err = a.arg(0).clone();
        crate::extras::bed_mesh::verify_algorithm(&mut b.mesh_config.borrow_mut(), &error_fn(&err))?;
        Ok(py_none())
    });
    method!(r, "klippy.extras.bed_mesh.BedMeshCalibrate", BedMeshCalibrate, set_adaptive_mesh(b, g: PyRef) => b.set_adaptive_mesh(&*gcmd(&g)?));
    method!(r, "klippy.extras.bed_mesh.BedMeshCalibrate", BedMeshCalibrate, update_config(b, g: PyRef) => b.update_config(&*gcmd(&g)?));
    method!(r, "klippy.extras.bed_mesh.BedMeshCalibrate", BedMeshCalibrate, _get_adjusted_points(b) => b.get_adjusted_points());
    method!(r, "klippy.extras.bed_mesh.BedMeshCalibrate", BedMeshCalibrate, cmd_BED_MESH_CALIBRATE(b, g: PyRef) =>
        b.cmd_bed_mesh_calibrate(&*gcmd(&g)?));
    // probe_finalize(offsets, positions)
    r.method("klippy.extras.bed_mesh.BedMeshCalibrate.probe_finalize", |a| {
        let o: Vec<f64> = FromPy::from_py(a.arg(0))?;
        let positions: Vec<Vec<f64>> = FromPy::from_py(a.arg(1))?;
        Ok(finalize_to_py(a.this::<BedMeshCalibrate>()?.probe_finalize((o[0], o[1], o[2]), &positions)?))
    });
    r.method("klippy.extras.bed_mesh.BedMeshCalibrate._dump_points", |a| {
        let probed: Vec<Vec<f64>> = FromPy::from_py(a.arg(0))?;
        let corrected: Vec<Vec<f64>> = FromPy::from_py(a.arg(1))?;
        let o: Vec<f64> = FromPy::from_py(a.arg(2))?;
        a.this::<BedMeshCalibrate>()?.dump_points(&probed, &corrected, (o[0], o[1], o[2]));
        Ok(py_none())
    });
    ro_attr!(r, "klippy.extras.bed_mesh.BedMeshCalibrate.printer", BedMeshCalibrate, |b| b.printer.clone());
    ro_attr!(r, "klippy.extras.bed_mesh.BedMeshCalibrate.gcode", BedMeshCalibrate, |b| b.gcode.clone());
    ro_attr!(r, "klippy.extras.bed_mesh.BedMeshCalibrate.bedmesh", BedMeshCalibrate, |b| b.bedmesh.upgrade());
    ro_attr!(r, "klippy.extras.bed_mesh.BedMeshCalibrate.adaptive_margin", BedMeshCalibrate, |b| b.adaptive_margin);
    ro_attr!(r, "klippy.extras.bed_mesh.BedMeshCalibrate.probe_helper", BedMeshCalibrate, |b| b.probe_helper.borrow().clone());
    cell_attr!(r, "klippy.extras.bed_mesh.BedMeshCalibrate.radius", BedMeshCalibrate, radius);
    cell_attr!(r, "klippy.extras.bed_mesh.BedMeshCalibrate.origin", BedMeshCalibrate, origin);
    cell_attr!(r, "klippy.extras.bed_mesh.BedMeshCalibrate.mesh_min", BedMeshCalibrate, mesh_min);
    cell_attr!(r, "klippy.extras.bed_mesh.BedMeshCalibrate.mesh_max", BedMeshCalibrate, mesh_max);
    cell_attr!(r, "klippy.extras.bed_mesh.BedMeshCalibrate.zero_ref_pos", BedMeshCalibrate, zero_ref_pos);
    vec_attr!(r, "klippy.extras.bed_mesh.BedMeshCalibrate.points", BedMeshCalibrate, points);
    r.attr("klippy.extras.bed_mesh.BedMeshCalibrate._profile_name", |o| bmc(o)?.profile_name.borrow().clone().to_py(), Some(|o, v| {
        *bmc(o)?.profile_name.borrow_mut() = FromPy::from_py(v)?;
        Ok(())
    }));
    r.attr("klippy.extras.bed_mesh.BedMeshCalibrate.zero_reference_mode", |o| Ok(py_int(zref_to_i64(bmc(o)?.zero_reference_mode.get()))), None);
    r.attr("klippy.extras.bed_mesh.BedMeshCalibrate.faulty_regions", |o| {
        let fr = bmc(o)?.faulty_regions.clone();
        Ok(py_list(fr.into_iter().map(|(a, b)| py_tuple(vec![pair_to_py(a), pair_to_py(b)])).collect()))
    }, None);
    r.attr("klippy.extras.bed_mesh.BedMeshCalibrate.substituted_indices", |o| {
        let d = lookup("builtins", "dict")?.call(&[])?;
        for (i, pts) in bmc(o)?.substituted_indices.borrow().iter() {
            d.call_method("__setitem__", &[&py_int(*i as i64), &py_list(pts.iter().map(|p| pair_to_py(*p)).collect())])?;
        }
        Ok(d)
    }, None);
    // orig_config: {"radius", "origin", "mesh_min", "mesh_max", "mesh_config"}
    r.attr("klippy.extras.bed_mesh.BedMeshCalibrate.orig_config", |o| {
        let oc = bmc(o)?.orig_config.clone();
        let d = lookup("builtins", "dict")?.call(&[])?;
        d.call_method("__setitem__", &[&py_str("radius"), &oc.radius.to_py()?])?;
        d.call_method("__setitem__", &[&py_str("origin"), &oc.origin.to_py()?])?;
        d.call_method("__setitem__", &[&py_str("mesh_min"), &pair_to_py(oc.mesh_min)])?;
        d.call_method("__setitem__", &[&py_str("mesh_max"), &pair_to_py(oc.mesh_max)])?;
        d.call_method("__setitem__", &[&py_str("mesh_config"), &params_to_py(&oc.mesh_config)])?;
        Ok(d)
    }, None);
    r.dict("klippy.extras.bed_mesh.BedMeshCalibrate.mesh_config", DictField {
        get: |o, _f, key| match bmc(o)?.mesh_config.borrow().get(key.as_string()?.as_str()) {
            Some(v) => Ok(from_pyval(v)),
            None => Err(KError::Py("KeyError".into(), key.repr()?)),
        },
        set: |o, _f, key, value| {
            let v = to_pyval(value)?;
            bmc(o)?.mesh_config.borrow_mut().insert(key.as_string()?.into(), v);
            Ok(())
        },
        del: |o, _f, key| match bmc(o)?.mesh_config.borrow_mut().shift_remove(key.as_string()?.as_str()) {
            Some(_) => Ok(()),
            None => Err(KError::Py("KeyError".into(), key.repr()?)),
        },
        keys: |o, _f| Ok(bmc(o)?.mesh_config.borrow().keys().map(|k| py_str(k)).collect()),
    });

    // -------------------------------------------------------------- ZMesh
    // ZMesh(params, name)
    r.ctor("klippy.extras.bed_mesh.ZMesh.__init__", |a| {
        let name: Option<String> = FromPy::from_py(a.arg(1))?;
        let z: Rc<dyn Any> = Rc::new(ZMesh::new(params_from_py(a.arg(0))?, name.as_deref()));
        Ok(z)
    });
    method!(r, "klippy.extras.bed_mesh.ZMesh", ZMesh, get_mesh_matrix(z) => z.get_mesh_matrix());
    method!(r, "klippy.extras.bed_mesh.ZMesh", ZMesh, get_probed_matrix(z) => z.get_probed_matrix());
    r.method("klippy.extras.bed_mesh.ZMesh.get_mesh_params", |a| Ok(params_to_py(&a.this::<ZMesh>()?.get_mesh_params())));
    method!(r, "klippy.extras.bed_mesh.ZMesh", ZMesh, get_profile_name(z) => z.get_profile_name());
    r.method("klippy.extras.bed_mesh.ZMesh.print_probed_matrix", |a| {
        let f = a.arg(0).clone();
        a.this::<ZMesh>()?.print_probed_matrix(&print_func(&f));
        Ok(py_none())
    });
    // print_mesh(print_func, move_z=None)
    r.method("klippy.extras.bed_mesh.ZMesh.print_mesh", |a| {
        let f = a.arg(0).clone();
        a.this::<ZMesh>()?.print_mesh(&print_func(&f), a.opt_f64(1)?);
        Ok(py_none())
    });
    r.method("klippy.extras.bed_mesh.ZMesh.build_mesh", |a| {
        let m: Vec<Vec<f64>> = FromPy::from_py(a.arg(0))?;
        a.this::<ZMesh>()?.build_mesh(m).map_err(mesh_error)?;
        Ok(py_none())
    });
    method!(r, "klippy.extras.bed_mesh.ZMesh", ZMesh, set_zero_reference(z, xpos: f64, ypos: f64) => z.set_zero_reference(xpos, ypos));
    method!(r, "klippy.extras.bed_mesh.ZMesh", ZMesh, set_mesh_offsets(z, offsets: Vec<Option<f64>>) => z.set_mesh_offsets(&offsets));
    method!(r, "klippy.extras.bed_mesh.ZMesh", ZMesh, get_x_coordinate(z, index: usize) => z.get_x_coordinate(index));
    method!(r, "klippy.extras.bed_mesh.ZMesh", ZMesh, get_y_coordinate(z, index: usize) => z.get_y_coordinate(index));
    method!(r, "klippy.extras.bed_mesh.ZMesh", ZMesh, calc_z(z, x: f64, y: f64) => z.calc_z(x, y));
    method!(r, "klippy.extras.bed_mesh.ZMesh", ZMesh, get_z_range(z) => z.get_z_range());
    method!(r, "klippy.extras.bed_mesh.ZMesh", ZMesh, get_z_average(z) => z.get_z_average());
    method!(r, "klippy.extras.bed_mesh.ZMesh", ZMesh, _get_linear_index(z, coord: f64, axis: usize) => z.get_linear_index(coord, axis));
    // _sample_direct(z_matrix): the mesh is the probed matrix
    r.method("klippy.extras.bed_mesh.ZMesh._sample_direct", |a| {
        let m: Vec<Vec<f64>> = FromPy::from_py(a.arg(0))?;
        *a.this::<ZMesh>()?.mesh_matrix.borrow_mut() = Some(m);
        Ok(py_none())
    });
    r.method("klippy.extras.bed_mesh.ZMesh._sample_lagrange", |a| {
        let m: Vec<Vec<f64>> = FromPy::from_py(a.arg(0))?;
        a.this::<ZMesh>()?.sample_lagrange(&m);
        Ok(py_none())
    });
    method!(r, "klippy.extras.bed_mesh.ZMesh", ZMesh, _get_lagrange_coords(z) => z.get_lagrange_coords());
    // _calc_lagrange(lpts, c, vec, axis=0): on self.mesh_matrix
    r.method("klippy.extras.bed_mesh.ZMesh._calc_lagrange", |a| {
        let z = a.this::<ZMesh>()?;
        let lpts: Vec<f64> = FromPy::from_py(a.arg(0))?;
        let mm = z.mesh_matrix.borrow().clone().unwrap_or_default();
        Ok(py_float(z.calc_lagrange(&mm, &lpts, a.f64(1)?, a.arg(2).as_i64()? as usize, a.arg(3).as_i64()? as usize)))
    });
    r.method("klippy.extras.bed_mesh.ZMesh._sample_bicubic", |a| {
        let m: Vec<Vec<f64>> = FromPy::from_py(a.arg(0))?;
        a.this::<ZMesh>()?.sample_bicubic(&m).map_err(mesh_error)?;
        Ok(py_none())
    });
    r.method("klippy.extras.bed_mesh.ZMesh._get_x_ctl_pts", |a| {
        let z = a.this::<ZMesh>()?;
        let mm = z.mesh_matrix.borrow().clone().unwrap_or_default();
        z.get_x_ctl_pts(&mm, a.arg(0).as_i64()? as usize, a.arg(1).as_i64()? as usize).map_err(mesh_error)?.to_vec().to_py()
    });
    r.method("klippy.extras.bed_mesh.ZMesh._get_y_ctl_pts", |a| {
        let z = a.this::<ZMesh>()?;
        let mm = z.mesh_matrix.borrow().clone().unwrap_or_default();
        z.get_y_ctl_pts(&mm, a.arg(0).as_i64()? as usize, a.arg(1).as_i64()? as usize).map_err(mesh_error)?.to_vec().to_py()
    });
    // _cardinal_spline(p, tension)
    r.method("klippy.extras.bed_mesh.ZMesh._cardinal_spline", |a| {
        let p: Vec<f64> = FromPy::from_py(a.arg(0))?;
        let p: [f64; 5] = p.try_into().map_err(|_| KError::Py("IndexError".into(), "list index out of range".into()))?;
        Ok(py_float(ZMesh::cardinal_spline(p, a.f64(1)?)))
    });
    // self._sample = the sampling method of the algorithm
    r.attr("klippy.extras.bed_mesh.ZMesh._sample", |o| {
        let algo = zm(o)?.algo.clone();
        o.getattr(&format!("_sample_{}", algo))
    }, None);
    r.attr("klippy.extras.bed_mesh.ZMesh.mesh_matrix", |o| zm(o)?.mesh_matrix.borrow().clone().to_py(), Some(|o, v| {
        *zm(o)?.mesh_matrix.borrow_mut() = FromPy::from_py(v)?;
        Ok(())
    }));
    r.attr("klippy.extras.bed_mesh.ZMesh.probed_matrix", |o| zm(o)?.probed_matrix.borrow().clone().to_py(), Some(|o, v| {
        *zm(o)?.probed_matrix.borrow_mut() = FromPy::from_py(v)?;
        Ok(())
    }));
    r.attr("klippy.extras.bed_mesh.ZMesh.mesh_offsets", |o| zm(o)?.mesh_offsets.borrow().to_vec().to_py(), None);
    r.attr("klippy.extras.bed_mesh.ZMesh.mesh_params", |o| Ok(params_to_py(&zm(o)?.mesh_params)), None);
    ro_attr!(r, "klippy.extras.bed_mesh.ZMesh.profile_name", ZMesh, |z| z.profile_name.clone());
    ro_attr!(r, "klippy.extras.bed_mesh.ZMesh.mesh_x_min", ZMesh, |z| z.mesh_x_min);
    ro_attr!(r, "klippy.extras.bed_mesh.ZMesh.mesh_y_min", ZMesh, |z| z.mesh_y_min);
    // (self.mesh_x_max = params["max_x"], ...)
    r.attr("klippy.extras.bed_mesh.ZMesh.mesh_x_max", |o| Ok(from_pyval(zm(o)?.mesh_params.get("max_x").unwrap_or(&PyVal::None))), None);
    r.attr("klippy.extras.bed_mesh.ZMesh.mesh_y_max", |o| Ok(from_pyval(zm(o)?.mesh_params.get("max_y").unwrap_or(&PyVal::None))), None);
    ro_attr!(r, "klippy.extras.bed_mesh.ZMesh.mesh_x_count", ZMesh, |z| z.mesh_x_count);
    ro_attr!(r, "klippy.extras.bed_mesh.ZMesh.mesh_y_count", ZMesh, |z| z.mesh_y_count);
    ro_attr!(r, "klippy.extras.bed_mesh.ZMesh.x_mult", ZMesh, |z| z.x_mult);
    ro_attr!(r, "klippy.extras.bed_mesh.ZMesh.y_mult", ZMesh, |z| z.y_mult);
    ro_attr!(r, "klippy.extras.bed_mesh.ZMesh.mesh_x_dist", ZMesh, |z| z.mesh_x_dist);
    ro_attr!(r, "klippy.extras.bed_mesh.ZMesh.mesh_y_dist", ZMesh, |z| z.mesh_y_dist);

    // ------------------------------------------------------- MoveSplitter
    r.ctor("klippy.extras.bed_mesh.MoveSplitter.__init__", |a| {
        let gcode = objects::unwrap_as::<crate::gcode::GCodeDispatch>(a.arg(1))?;
        let s: Rc<dyn Any> = Rc::new(MoveSplitter::new(&*cfg(a.arg(0))?, &gcode)?);
        Ok(s)
    });
    method!(r, "klippy.extras.bed_mesh.MoveSplitter", MoveSplitter, initialize(s, mesh: PyRef, fade_offset: f64) =>
        s.initialize(opt_mesh_from_py(&mesh)?, fade_offset));
    method!(r, "klippy.extras.bed_mesh.MoveSplitter", MoveSplitter, build_move(s, prev_pos: Vec<f64>, next_pos: Vec<f64>, factor: f64) =>
        s.build_move(&prev_pos, &next_pos, factor));
    method!(r, "klippy.extras.bed_mesh.MoveSplitter", MoveSplitter, _calc_z_offset(s, pos: Vec<f64>) => s.calc_z_offset(&pos));
    method!(r, "klippy.extras.bed_mesh.MoveSplitter", MoveSplitter, _set_next_move(s, distance_from_prev: f64) => s.set_next_move(distance_from_prev));
    method!(r, "klippy.extras.bed_mesh.MoveSplitter", MoveSplitter, split(s) => s.split());
    ro_attr!(r, "klippy.extras.bed_mesh.MoveSplitter.split_delta_z", MoveSplitter, |s| s.split_delta_z);
    ro_attr!(r, "klippy.extras.bed_mesh.MoveSplitter.move_check_distance", MoveSplitter, |s| s.move_check_distance);
    r.attr("klippy.extras.bed_mesh.MoveSplitter.z_mesh", |o| ms(o)?.z_mesh.borrow().clone().to_py(), None);
    r.attr("klippy.extras.bed_mesh.MoveSplitter.gcode", |o| {
        let _ = ms(o)?;
        printer().ok_or_else(|| KError::Other("kaliru: no printer".into()))?.lookup::<crate::gcode::GCodeDispatch>("gcode")?.to_py()
    }, None);
    cell_attr!(r, "klippy.extras.bed_mesh.MoveSplitter.fade_offset", MoveSplitter, fade_offset);
    cell_attr!(r, "klippy.extras.bed_mesh.MoveSplitter.z_factor", MoveSplitter, z_factor);
    cell_attr!(r, "klippy.extras.bed_mesh.MoveSplitter.z_offset", MoveSplitter, z_offset);
    cell_attr!(r, "klippy.extras.bed_mesh.MoveSplitter.traverse_complete", MoveSplitter, traverse_complete);
    cell_attr!(r, "klippy.extras.bed_mesh.MoveSplitter.distance_checked", MoveSplitter, distance_checked);
    cell_attr!(r, "klippy.extras.bed_mesh.MoveSplitter.total_move_length", MoveSplitter, total_move_length);
    vec_attr!(r, "klippy.extras.bed_mesh.MoveSplitter.prev_pos", MoveSplitter, prev_pos);
    vec_attr!(r, "klippy.extras.bed_mesh.MoveSplitter.next_pos", MoveSplitter, next_pos);
    vec_attr!(r, "klippy.extras.bed_mesh.MoveSplitter.current_pos", MoveSplitter, current_pos);
    vec_attr!(r, "klippy.extras.bed_mesh.MoveSplitter.axis_move", MoveSplitter, axis_move);

    // ----------------------------------------------------- ProfileManager
    r.ctor("klippy.extras.bed_mesh.ProfileManager.__init__", |a| {
        let p: Rc<dyn Any> = ProfileManager::new(&*cfg(a.arg(0))?, &bm(a.arg(1))?)?;
        Ok(p)
    });
    r.method("klippy.extras.bed_mesh.ProfileManager.get_profiles", |a| Ok(from_pyval(&a.this::<ProfileManager>()?.profiles_pyval())));
    method!(r, "klippy.extras.bed_mesh.ProfileManager", ProfileManager, _check_incompatible_profiles(p) => p.check_incompatible_profiles());
    method!(r, "klippy.extras.bed_mesh.ProfileManager", ProfileManager, save_profile(p, prof_name: String) => p.save_profile(&prof_name));
    method!(r, "klippy.extras.bed_mesh.ProfileManager", ProfileManager, load_profile(p, prof_name: String) => p.load_profile(&prof_name));
    method!(r, "klippy.extras.bed_mesh.ProfileManager", ProfileManager, remove_profile(p, prof_name: String) => p.remove_profile(&prof_name));
    method!(r, "klippy.extras.bed_mesh.ProfileManager", ProfileManager, cmd_BED_MESH_PROFILE(p, g: PyRef) => p.cmd_bed_mesh_profile(&*gcmd(&g)?));
    ro_attr!(r, "klippy.extras.bed_mesh.ProfileManager.name", ProfileManager, |p| p.name.clone());
    ro_attr!(r, "klippy.extras.bed_mesh.ProfileManager.printer", ProfileManager, |p| p.printer.clone());
    ro_attr!(r, "klippy.extras.bed_mesh.ProfileManager.gcode", ProfileManager, |p| p.gcode.clone());
    ro_attr!(r, "klippy.extras.bed_mesh.ProfileManager.bedmesh", ProfileManager, |p| p.bedmesh.upgrade());
    ro_attr!(r, "klippy.extras.bed_mesh.ProfileManager.incompatible_profiles", ProfileManager, |p| p.incompatible_profiles.clone());
    r.attr("klippy.extras.bed_mesh.ProfileManager.profiles", |o| {
        Ok(from_pyval(&objects::unwrap_as::<ProfileManager>(o)?.profiles_pyval()))
    }, None);
}

