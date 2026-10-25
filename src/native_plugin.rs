// Native plugins: crates a distribution compiles into its kaliru binary
// (docs/PLUGINS.md "Native plugins"). A plugin brings printer modules; it
// runs when the user's config has its sections, like kaliru's own modules,
// and a config its active() check turns down leaves it out (the python
// modules of the same names then load as before).
use crate::configfile::ConfigWrapper;
use crate::error::KResult;
use crate::printer::PrinterModule;
use std::sync::OnceLock;

pub struct NativePlugin {
    pub name: &'static str,
    pub modules: &'static [PrinterModule],
    /// whether this config runs the plugin: its modules then win over python
    /// modules (klippy/extras, klippy/plugins) of the same names
    pub active: fn(&ConfigWrapper) -> KResult<bool>,
}

static PLUGINS: OnceLock<&'static [NativePlugin]> = OnceLock::new();

/// the plugins compiled into this binary
pub fn plugins() -> &'static [NativePlugin] {
    PLUGINS.get().copied().unwrap_or(&[])
}

/// A distribution's main: kaliru with these plugins
///   fn main() { kaliru::native_plugin::main(&[rust_afc::PLUGIN]) }
pub fn main(plugins: &'static [NativePlugin]) {
    let _ = PLUGINS.set(plugins);
    crate::printer::main();
}
