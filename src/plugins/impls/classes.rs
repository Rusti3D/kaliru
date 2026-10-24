// Kalico classes of kaliru printer objects whose Rust type path does not
// name them (a plugin iterating printer.lookup_objects() meets them all)
use super::*;

pub fn register(_r: &mut Registry) {
    use crate::extras as x;
    objects::register_class::<x::ad5206::Ad5206>("klippy.extras.ad5206", "ad5206");
    objects::register_class::<x::dac084s085::Dac084S085>("klippy.extras.dac084s085", "dac084S085");
    objects::register_class::<x::display::PrinterLCD>("klippy.extras.display.display", "PrinterLCD");
    objects::register_class::<x::display::menu::MenuObject>("klippy.extras.display.menu", "MenuManager");
    objects::register_class::<x::display_templates::PrinterDisplayTemplate>("klippy.extras.display.display", "PrinterDisplayTemplate");
    objects::register_class::<x::error_mcu::PrinterMcuError>("klippy.extras.error_mcu", "PrinterMCUError");
    objects::register_class::<x::heated_fan::HeatedFanObject>("klippy.extras.heated_fan", "HeatedFan");
    objects::register_class::<x::mcp4018::Mcp4018>("klippy.extras.mcp4018", "mcp4018");
    objects::register_class::<x::mcp4451::Mcp4451>("klippy.extras.mcp4451", "mcp4451");
    objects::register_class::<x::mcp4728::Mcp4728>("klippy.extras.mcp4728", "mcp4728");
    objects::register_class::<x::telemetry::KalicoTelemetry>("klippy.extras.telemetry", "KalicoTelementry");
    objects::register_class::<x::temperature_host::TemperatureHost>("klippy.extras.temperature_host", "Temperature_HOST");
}
