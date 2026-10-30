// Kalico classes of kaliru printer objects whose Rust type path does not
// name them (a plugin iterating printer.lookup_objects() meets them all)
use super::*;

pub fn register(_r: &mut Registry) {
    use crate::extras as x;
    objects::register_class::<x::ad5206::Ad5206>("klippy.extras.ad5206", "ad5206");
    objects::register_class::<x::dac084s085::Dac084S085>("klippy.extras.dac084s085", "dac084S085");
    objects::register_class::<x::display::PrinterLCD>("klippy.extras.display.display", "PrinterLCD");
    objects::register_class::<x::stepper_enable::StepperEnablePin>("klippy.extras.stepper_enable", "StepperEnablePin");
    objects::register_class::<x::tmc_uart::McuTmcUartBitbang>("klippy.extras.tmc_uart", "MCU_TMC_uart_bitbang");
    // the "menu" object: python's is the MenuManager itself
    objects::register_redirect::<x::display::menu::MenuObject>(|a| {
        let m: Rc<dyn Any> = a.downcast_ref::<x::display::menu::MenuObject>()?.manager()?;
        Some((m, std::any::type_name::<x::display::menu::MenuManager>()))
    });
    objects::register_class_fn::<x::display::menu::MenuItem>(|a| {
        use x::display::menu::Kind;
        let name = match a.downcast_ref::<x::display::menu::MenuItem>().map(|m| &m.kind) {
            Some(Kind::Disabled) => "MenuDisabled",
            Some(Kind::Command) => "MenuCommand",
            Some(Kind::Input) => "MenuInput",
            Some(Kind::List) => "MenuList",
            Some(Kind::Dialog) => "MenuDialog",
            Some(Kind::VsdList) => "MenuVSDList",
            _ => "MenuFileBrowser",
        };
        ("klippy.extras.display.menu", name)
    });
    objects::register_class::<x::display_templates::PrinterDisplayTemplate>("klippy.extras.display.display", "PrinterDisplayTemplate");
    objects::register_class::<x::display_templates::DisplayTemplate>("klippy.extras.display.display", "DisplayTemplate");
    objects::register_class::<x::display_templates::DisplayGroup>("klippy.extras.display.display", "DisplayGroup");
    objects::register_class::<x::error_mcu::PrinterMcuError>("klippy.extras.error_mcu", "PrinterMCUError");
    objects::register_class::<x::heated_fan::HeatedFanObject>("klippy.extras.heated_fan", "HeatedFan");
    objects::register_class::<x::mcp4018::Mcp4018>("klippy.extras.mcp4018", "mcp4018");
    objects::register_class::<x::mcp4451::Mcp4451>("klippy.extras.mcp4451", "mcp4451");
    objects::register_class::<x::mcp4728::Mcp4728>("klippy.extras.mcp4728", "mcp4728");
    objects::register_class::<x::telemetry::KalicoTelemetry>("klippy.extras.telemetry", "KalicoTelementry");
    objects::register_class::<x::temperature_host::TemperatureHost>("klippy.extras.temperature_host", "Temperature_HOST");

    // one Rust type for several Kalico classes: the class of the object's kind
    objects::register_class_fn::<x::aht10::AHTBase>(|a| match a.downcast_ref::<x::aht10::AHTBase>().map(|s| &s.model) {
        Some(x::aht10::Model::Aht1x) => ("klippy.extras.aht10", "AHT1x"),
        Some(x::aht10::Model::Aht2x) => ("klippy.extras.aht10", "AHT2x"),
        _ => ("klippy.extras.aht10", "AHT3x"),
    });
    objects::register_class_fn::<x::spi_temperature::SensorBase>(|a| {
        use x::spi_temperature::Chip;
        match a.downcast_ref::<x::spi_temperature::SensorBase>().map(|s| &s.chip) {
            Some(Chip::Max6675) => ("klippy.extras.spi_temperature", "MAX6675"),
            Some(Chip::Max31855) => ("klippy.extras.spi_temperature", "MAX31855"),
            Some(Chip::Max31856) => ("klippy.extras.spi_temperature", "MAX31856"),
            _ => ("klippy.extras.spi_temperature", "MAX31865"),
        }
    });
    objects::register_class_fn::<x::display::st7920::DisplayBase>(|a| match a.downcast_ref::<x::display::st7920::DisplayBase>().map(|d| &d.io) {
        Some(x::display::st7920::Io::Native { .. }) => ("klippy.extras.display.st7920", "ST7920"),
        _ => ("klippy.extras.display.st7920", "EmulatedST7920"),
    });
    // (the SH1106 is the SSD1306 with 132 columns)
    objects::register_class_fn::<x::display::uc1701::DisplayBase>(|a| {
        let d = a.downcast_ref::<x::display::uc1701::DisplayBase>().expect("type");
        match &d.chip {
            x::display::uc1701::Chip::UC1701 { .. } => ("klippy.extras.display.uc1701", "UC1701"),
            _ if d.vram.borrow()[0].0.len() == 132 => ("klippy.extras.display.uc1701", "SH1106"),
            _ => ("klippy.extras.display.uc1701", "SSD1306"),
        }
    });
    objects::register_class_fn::<x::load_cell::hx71x::HX71xBase>(|a| match a.downcast_ref::<x::load_cell::hx71x::HX71xBase>().map(|s| s.sensor_type) {
        Some("hx717") => ("klippy.extras.load_cell.hx71x", "HX717"),
        _ => ("klippy.extras.load_cell.hx71x", "HX711"),
    });
    objects::register_class_fn::<x::load_cell::ads131m0x::ADS131MxBase>(|a| match a.downcast_ref::<x::load_cell::ads131m0x::ADS131MxBase>().map(|s| s.channel_count) {
        Some(2) => ("klippy.extras.load_cell.ads131m0x", "ADS131M02"),
        _ => ("klippy.extras.load_cell.ads131m0x", "ADS131M04"),
    });
    objects::register_class_fn::<x::mpc_block_temperature::MpcTempWrapper>(|a| {
        match a.downcast_ref::<x::mpc_block_temperature::MpcTempWrapper>().map(|w| &w.kind) {
            Some(x::mpc_block_temperature::Kind::Block) => ("klippy.extras.mpc_block_temperature", "MPC_BLOCK_TEMP_WRAPPER"),
            _ => ("klippy.extras.mpc_ambient_temperature", "MPC_AMBIENT_TEMP_WRAPPER"),
        }
    });
    objects::register_class::<x::mixing_extruder::MixingExtruder>("klippy.extras.mixing_extruder", "GradientMixingExtruder");
    objects::register_class::<crate::clocksync::Secondary>("klippy.clocksync", "SecondarySync");
}
