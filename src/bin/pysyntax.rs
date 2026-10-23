// Prints the SyntaxError kaliru reports for each python source in a JSON
// list on stdin, one JSON [class, msg, lineno, offset] (or null) per line;
// with --exec, the exception running it raises as [class, msg]. Twin of
// tools/gen_pysyntax_cases.py. Development tool.
#[cfg(feature = "python")]
fn main() {
    use kaliru::extras::gcode_python::{exec_error, syntax_error};
    use std::io::Read;
    let exec = std::env::args().any(|a| a == "--exec");
    let mut input = String::new();
    std::io::stdin().read_to_string(&mut input).unwrap();
    let sources: Vec<String> = serde_json::from_str(&input).unwrap();
    for src in sources {
        let out = if exec {
            exec_error(&src).map(|(cls, msg)| serde_json::json!([cls, msg]))
        } else {
            syntax_error(&src).map(|(cls, msg, line, col)| serde_json::json!([cls, msg, line, col]))
        };
        println!("{}", out.unwrap_or(serde_json::Value::Null));
    }
}

#[cfg(not(feature = "python"))]
fn main() {
    eprintln!("built without the python feature");
}
