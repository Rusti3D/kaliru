use crate::{
    builtins::PyBaseExceptionRef,
    convert::{IntoPyException, ToPyException},
    function::FuncArgs,
    stdlib::builtins,
    PyObject, PyResult, VirtualMachine,
};

use rustpython_format::*;

impl IntoPyException for FormatSpecError {
    fn into_pyexception(self, vm: &VirtualMachine) -> PyBaseExceptionRef {
        match self {
            FormatSpecError::DecimalDigitsTooMany => {
                vm.new_value_error("Too many decimal digits in format string".to_owned())
            }
            FormatSpecError::PrecisionTooBig => vm.new_value_error("Precision too big".to_owned()),
            FormatSpecError::InvalidFormatSpecifier => {
                vm.new_value_error("Invalid format specifier".to_owned())
            }
            FormatSpecError::UnspecifiedFormat(c1, c2) => {
                let msg = format!("Cannot specify '{c1}' with '{c2}'.");
                vm.new_value_error(msg)
            }
            FormatSpecError::UnknownFormatCode(c, s) => {
                let msg = format!("Unknown format code '{c}' for object of type '{s}'");
                vm.new_value_error(msg)
            }
            FormatSpecError::PrecisionNotAllowed => {
                vm.new_value_error("Precision not allowed in integer format specifier".to_owned())
            }
            FormatSpecError::NotAllowed(s) => {
                let msg = format!("{s} not allowed with integer format specifier 'c'");
                vm.new_value_error(msg)
            }
            FormatSpecError::UnableToConvert => {
                vm.new_value_error("Unable to convert int to float".to_owned())
            }
            FormatSpecError::CodeNotInRange => {
                vm.new_overflow_error("%c arg not in range(0x110000)".to_owned())
            }
            FormatSpecError::NotImplemented(c, s) => {
                let msg = format!("Format code '{c}' for object of type '{s}' not implemented yet");
                vm.new_value_error(msg)
            }
        }
    }
}

impl ToPyException for FormatParseError {
    fn to_pyexception(&self, vm: &VirtualMachine) -> PyBaseExceptionRef {
        match self {
            FormatParseError::UnmatchedBracket => {
                vm.new_value_error("expected '}' before end of string".to_owned())
            }
            _ => vm.new_value_error("Unexpected error parsing format string".to_owned()),
        }
    }
}

/// a FormatParseError of format string `s` in CPython's words
/// (Objects/stringlib/unicode_format.h)
pub(crate) fn parse_error(e: &FormatParseError, s: &str, vm: &VirtualMachine) -> PyBaseExceptionRef {
    // the first brace that is neither doubled nor part of a field
    let single = || {
        let c: Vec<char> = s.chars().collect();
        let mut i = 0;
        while i < c.len() {
            match c[i] {
                '{' if c.get(i + 1) == Some(&'{') => i += 2,
                '}' if c.get(i + 1) == Some(&'}') => i += 2,
                '{' => {
                    if i + 1 == c.len() {
                        return Some('{');
                    }
                    match c[i + 1..].iter().position(|&x| x == '}') {
                        Some(p) => i += p + 2,
                        None => return None,
                    }
                }
                '}' => return Some('}'),
                _ => i += 1,
            }
        }
        None
    };
    let msg = match e {
        FormatParseError::UnmatchedBracket
        | FormatParseError::UnescapedStartBracketInLiteral
        | FormatParseError::MissingStartBracket => match single() {
            Some(c) => format!("Single '{c}' encountered in format string"),
            None => "expected '}' before end of string".to_owned(),
        },
        FormatParseError::EmptyAttribute => "Empty attribute in format string".to_owned(),
        FormatParseError::MissingRightBracket => "Missing ']' in format string".to_owned(),
        FormatParseError::InvalidCharacterAfterRightBracket => {
            "Only '.' or '[' may follow ']' in format field specifier".to_owned()
        }
        _ => return e.to_pyexception(vm),
    };
    vm.new_value_error(msg)
}

/// a FormatSpecError of `spec` for an object of type `tname`
/// (Python/formatter_unicode.c parse_internal_render_format_spec)
pub(crate) fn spec_error(e: FormatSpecError, spec: &str, tname: &str, vm: &VirtualMachine) -> PyBaseExceptionRef {
    if let FormatSpecError::InvalidFormatSpecifier = e {
        // one character after a valid spec is the type code
        let mut prefix = spec.chars();
        let last = prefix.next_back();
        if let (Some(c), Ok(_)) = (last, FormatSpec::parse(prefix.as_str())) {
            return vm.new_value_error(format!("Unknown format code '{c}' for object of type '{tname}'"));
        }
        return vm.new_value_error(format!("Invalid format specifier '{spec}' for object of type '{tname}'"));
    }
    e.into_pyexception(vm)
}

fn format_internal(
    vm: &VirtualMachine,
    format: &FormatString,
    field_func: &mut impl FnMut(FieldType) -> PyResult,
) -> PyResult<String> {
    let mut final_string = String::new();
    for part in &format.format_parts {
        let pystr;
        let result_string: &str = match part {
            FormatPart::Field {
                field_name,
                conversion_spec,
                format_spec,
            } => {
                let FieldName { field_type, parts } =
                    FieldName::parse(field_name.as_str()).map_err(|e| e.to_pyexception(vm))?;

                let mut argument = field_func(field_type)?;

                for name_part in parts {
                    match name_part {
                        FieldNamePart::Attribute(attribute) => {
                            argument = argument.get_attr(&vm.ctx.new_str(attribute), vm)?;
                        }
                        FieldNamePart::Index(index) => {
                            argument = argument.get_item(&index, vm)?;
                        }
                        FieldNamePart::StringIndex(index) => {
                            argument = argument.get_item(&index, vm)?;
                        }
                    }
                }

                let nested_format =
                    FormatString::from_str(format_spec).map_err(|e| parse_error(&e, format_spec, vm))?;
                let format_spec = format_internal(vm, &nested_format, field_func)?;

                let argument = match conversion_spec.and_then(FormatConversion::from_char) {
                    Some(FormatConversion::Str) => argument.str(vm)?.into(),
                    Some(FormatConversion::Repr) => argument.repr(vm)?.into(),
                    Some(FormatConversion::Ascii) => {
                        vm.ctx.new_str(builtins::ascii(argument, vm)?).into()
                    }
                    Some(FormatConversion::Bytes) => {
                        vm.call_method(&argument, identifier!(vm, decode).as_str(), ())?
                    }
                    None => argument,
                };

                // FIXME: compiler can intern specs using parser tree. Then this call can be interned_str
                pystr = vm.format(&argument, vm.ctx.new_str(format_spec))?;
                pystr.as_ref()
            }
            FormatPart::Literal(literal) => literal,
        };
        final_string.push_str(result_string);
    }
    Ok(final_string)
}

pub(crate) fn format(
    format: &FormatString,
    arguments: &FuncArgs,
    vm: &VirtualMachine,
) -> PyResult<String> {
    let mut auto_argument_index: usize = 0;
    let mut seen_index = false;
    format_internal(vm, format, &mut |field_type| match field_type {
        FieldType::Auto => {
            if seen_index {
                return Err(vm.new_value_error(
                    "cannot switch from manual field specification to automatic field numbering"
                        .to_owned(),
                ));
            }
            auto_argument_index += 1;
            arguments
                .args
                .get(auto_argument_index - 1)
                .cloned()
                .ok_or_else(|| {
                    vm.new_index_error(format!(
                        "Replacement index {} out of range for positional args tuple",
                        auto_argument_index - 1
                    ))
                })
        }
        FieldType::Index(index) => {
            if auto_argument_index != 0 {
                return Err(vm.new_value_error(
                    "cannot switch from automatic field numbering to manual field specification"
                        .to_owned(),
                ));
            }
            seen_index = true;
            arguments
                .args
                .get(index)
                .cloned()
                .ok_or_else(|| {
                    vm.new_index_error(format!(
                        "Replacement index {index} out of range for positional args tuple"
                    ))
                })
        }
        FieldType::Keyword(keyword) => arguments
            .get_optional_kwarg(&keyword)
            .ok_or_else(|| vm.new_key_error(vm.ctx.new_str(keyword).into())),
    })
}

pub(crate) fn format_map(
    format: &FormatString,
    dict: &PyObject,
    vm: &VirtualMachine,
) -> PyResult<String> {
    format_internal(vm, format, &mut |field_type| match field_type {
        FieldType::Auto | FieldType::Index(_) => {
            Err(vm.new_value_error("Format string contains positional fields".to_owned()))
        }
        FieldType::Keyword(keyword) => dict.get_item(&keyword, vm),
    })
}
