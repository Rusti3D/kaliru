// Ports of the python 3.13 stdlib path helpers klippy uses: glob.glob (with
// recursive), glob.has_magic, fnmatch matching and the posixpath functions
// join, split, dirname, normpath, abspath (sources in kaliru/ref/).
use std::path::Path;

// ------------------------------------------------------------ posixpath

pub fn join(a: &str, parts: &[&str]) -> String {
    let mut path = a.to_string();
    for b in parts {
        if b.starts_with('/') || path.is_empty() {
            path = b.to_string();
        } else if path.ends_with('/') {
            path.push_str(b);
        } else {
            path.push('/');
            path.push_str(b);
        }
    }
    path
}

pub fn split(p: &str) -> (String, String) {
    let i = p.rfind('/').map(|i| i + 1).unwrap_or(0);
    let (mut head, tail) = (p[..i].to_string(), p[i..].to_string());
    if !head.is_empty() && head.chars().any(|c| c != '/') {
        head = head.trim_end_matches('/').to_string();
    }
    (head, tail)
}

pub fn dirname(p: &str) -> String {
    split(p).0
}

pub fn normpath(path: &str) -> String {
    if path.is_empty() {
        return ".".into();
    }
    let mut initial_slashes = if path.starts_with('/') { 1 } else { 0 };
    // POSIX allows one or two initial slashes, but treats three or more as single
    if path.starts_with("//") && !path.starts_with("///") {
        initial_slashes = 2;
    }
    let mut new: Vec<&str> = vec![];
    for comp in path.split('/') {
        if comp.is_empty() || comp == "." {
            continue;
        }
        if comp != ".." || (initial_slashes == 0 && new.is_empty()) || new.last() == Some(&"..") {
            new.push(comp);
        } else if !new.is_empty() {
            new.pop();
        }
    }
    let out = format!("{}{}", "/".repeat(initial_slashes), new.join("/"));
    if out.is_empty() {
        ".".into()
    } else {
        out
    }
}

/// tempfile.gettempdir()
pub fn gettempdir() -> String {
    for var in ["TMPDIR", "TEMP", "TMP"] {
        if let Ok(v) = std::env::var(var) {
            if !v.is_empty() && std::path::Path::new(&v).is_dir() {
                return v;
            }
        }
    }
    "/tmp".into()
}

/// posixpath.expanduser(path)
pub fn expanduser(path: &str) -> String {
    if !path.starts_with('~') {
        return path.to_string();
    }
    let i = path[1..].find('/').map(|i| i + 1).unwrap_or(path.len());
    let pw_dir = |pw: *mut libc::passwd| -> Option<String> {
        if pw.is_null() {
            return None;
        }
        // SAFETY: getpwuid / getpwnam returned a valid passwd entry
        Some(unsafe { std::ffi::CStr::from_ptr((*pw).pw_dir) }.to_string_lossy().into_owned())
    };
    let userhome = if i == 1 {
        match std::env::var("HOME") {
            Ok(h) => h,
            // SAFETY: plain libc lookups
            Err(_) => match pw_dir(unsafe { libc::getpwuid(libc::getuid()) }) {
                Some(h) => h,
                None => return path.to_string(),
            },
        }
    } else {
        let Ok(name) = std::ffi::CString::new(&path[1..i]) else { return path.to_string() };
        // SAFETY: plain libc lookup
        match pw_dir(unsafe { libc::getpwnam(name.as_ptr()) }) {
            Some(h) => h,
            None => return path.to_string(),
        }
    };
    let userhome = userhome.trim_end_matches('/');
    let out = format!("{}{}", userhome, &path[i..]);
    if out.is_empty() {
        "/".into()
    } else {
        out
    }
}

/// posixpath.expandvars(path): $name and ${name} from the environment
pub fn expandvars(path: &str) -> String {
    if !path.contains('$') {
        return path.to_string();
    }
    let mut path = path.to_string();
    let mut i = 0;
    loop {
        // re.compile(r'\$(\w+|\{[^}]*\})', re.ASCII).search(path, i)
        let bytes = path.as_bytes();
        let mut found = None;
        let mut k = i;
        while k < bytes.len() {
            if bytes[k] == b'$' {
                let start = k + 1;
                if start < bytes.len() && (bytes[start].is_ascii_alphanumeric() || bytes[start] == b'_') {
                    let mut e = start;
                    while e < bytes.len() && (bytes[e].is_ascii_alphanumeric() || bytes[e] == b'_') {
                        e += 1;
                    }
                    found = Some((k, e, path[start..e].to_string()));
                    break;
                }
                if start < bytes.len() && bytes[start] == b'{' {
                    if let Some(close) = path[start..].find('}') {
                        let e = start + close + 1;
                        found = Some((k, e, path[start + 1..e - 1].to_string()));
                        break;
                    }
                }
            }
            k += 1;
        }
        let Some((mi, j, name)) = found else { break };
        match std::env::var(&name) {
            Err(_) => i = j,
            Ok(value) => {
                let tail = path[j..].to_string();
                path = format!("{}{}", &path[..mi], value);
                i = path.len();
                path += &tail;
            }
        }
    }
    path
}

pub fn getcwd() -> String {
    std::env::current_dir().map(|p| p.to_string_lossy().into_owned()).unwrap_or_default()
}

pub fn abspath(path: &str) -> String {
    if path.starts_with('/') {
        normpath(path)
    } else {
        normpath(&join(&getcwd(), &[path]))
    }
}

// ------------------------------------------------------------ fnmatch

#[derive(Debug)]
enum Tok {
    Star,
    Any,
    Char(char),
    // (negated, items as inclusive ranges); an empty set never matches
    Set(bool, Vec<(char, char)>),
}

// fnmatch._translate: '[' without a closing ']' is a literal '['
fn translate(pat: &[char]) -> Vec<Tok> {
    let mut res = vec![];
    let (mut i, n) = (0, pat.len());
    while i < n {
        let c = pat[i];
        i += 1;
        match c {
            '*' => {
                if !matches!(res.last(), Some(Tok::Star)) {
                    res.push(Tok::Star);
                }
            }
            '?' => res.push(Tok::Any),
            '[' => {
                let mut j = i;
                if j < n && pat[j] == '!' {
                    j += 1;
                }
                if j < n && pat[j] == ']' {
                    j += 1;
                }
                while j < n && pat[j] != ']' {
                    j += 1;
                }
                if j >= n {
                    res.push(Tok::Char('['));
                    continue;
                }
                let mut stuff = &pat[i..j];
                i = j + 1;
                let negated = stuff.first() == Some(&'!');
                if negated {
                    stuff = &stuff[1..];
                }
                // ranges "a-z"; a range whose end is below its start is
                // empty and dropped (fnmatch "Remove empty ranges")
                // ponytail: python merges chunks around empty ranges in more
                // exotic sets ("a-c-e"); plain ranges and chars match exactly
                let mut items = vec![];
                let mut k = 0;
                while k < stuff.len() {
                    if k + 2 < stuff.len() && stuff[k + 1] == '-' {
                        if stuff[k] <= stuff[k + 2] {
                            items.push((stuff[k], stuff[k + 2]));
                        }
                        k += 3;
                    } else {
                        items.push((stuff[k], stuff[k]));
                        k += 1;
                    }
                }
                if items.is_empty() && negated {
                    // "[!]" style: negated empty range matches any character
                    res.push(Tok::Any);
                } else {
                    res.push(Tok::Set(negated, items));
                }
            }
            c => res.push(Tok::Char(c)),
        }
    }
    res
}

fn match_toks(toks: &[Tok], name: &[char]) -> bool {
    match toks.first() {
        None => name.is_empty(),
        Some(Tok::Star) => (0..=name.len()).any(|k| match_toks(&toks[1..], &name[k..])),
        Some(t) => {
            let Some(&c) = name.first() else { return false };
            let ok = match t {
                Tok::Any => true,
                Tok::Char(p) => *p == c,
                Tok::Set(neg, items) => {
                    if items.is_empty() {
                        false
                    } else {
                        items.iter().any(|(lo, hi)| *lo <= c && c <= *hi) != *neg
                    }
                }
                Tok::Star => unreachable!(),
            };
            ok && match_toks(&toks[1..], &name[1..])
        }
    }
}

/// fnmatch.fnmatchcase (posix: fnmatch == fnmatchcase)
pub fn fnmatch(name: &str, pat: &str) -> bool {
    let pat: Vec<char> = pat.chars().collect();
    let name: Vec<char> = name.chars().collect();
    match_toks(&translate(&pat), &name)
}

// ------------------------------------------------------------ glob

pub fn has_magic(s: &str) -> bool {
    s.contains(['*', '?', '['])
}

fn ishidden(path: &str) -> bool {
    path.starts_with('.')
}

fn isrecursive(pattern: &str) -> bool {
    pattern == "**"
}

fn join2(dirname: &str, basename: &str) -> String {
    if dirname.is_empty() || basename.is_empty() {
        return format!("{}{}", dirname, basename);
    }
    join(dirname, &[basename])
}

fn lexists(p: &str) -> bool {
    std::fs::symlink_metadata(p).is_ok()
}

fn isdir(p: &str) -> bool {
    Path::new(p).is_dir()
}

fn listdir(dirname: &str, dironly: bool) -> Vec<String> {
    let arg = if dirname.is_empty() { "." } else { dirname };
    let Ok(rd) = std::fs::read_dir(arg) else { return vec![] };
    let mut out = vec![];
    for e in rd.flatten() {
        // DirEntry.is_dir() follows symlinks, like os.scandir's
        if !dironly || e.path().is_dir() {
            out.push(e.file_name().to_string_lossy().into_owned());
        }
    }
    out
}

fn rlistdir(dirname: &str, dironly: bool) -> Vec<String> {
    let mut out = vec![];
    for x in listdir(dirname, dironly) {
        if !ishidden(&x) {
            let path = if dirname.is_empty() { x.clone() } else { join2(dirname, &x) };
            out.push(x.clone());
            for y in rlistdir(&path, dironly) {
                out.push(join2(&x, &y));
            }
        }
    }
    out
}

fn glob1(dirname: &str, pattern: &str, dironly: bool) -> Vec<String> {
    let mut names = listdir(dirname, dironly);
    if !ishidden(pattern) {
        names.retain(|x| !ishidden(x));
    }
    names.into_iter().filter(|n| fnmatch(n, pattern)).collect()
}

fn glob0(dirname: &str, basename: &str) -> Vec<String> {
    if !basename.is_empty() {
        if lexists(&join2(dirname, basename)) {
            return vec![basename.to_string()];
        }
    } else if isdir(dirname) {
        return vec![basename.to_string()];
    }
    vec![]
}

fn glob2(dirname: &str, dironly: bool) -> Vec<String> {
    let mut out = vec![];
    if dirname.is_empty() || isdir(dirname) {
        out.push(String::new());
    }
    out.extend(rlistdir(dirname, dironly));
    out
}

fn iglob_inner(pathname: &str, recursive: bool, dironly: bool) -> Vec<String> {
    let (dirname, basename) = split(pathname);
    if !has_magic(pathname) {
        if !basename.is_empty() {
            if lexists(pathname) {
                return vec![pathname.to_string()];
            }
        } else if isdir(&dirname) {
            return vec![pathname.to_string()];
        }
        return vec![];
    }
    if dirname.is_empty() {
        return if recursive && isrecursive(&basename) {
            glob2("", dironly)
        } else {
            glob1("", &basename, dironly)
        };
    }
    let dirs = if dirname != pathname && has_magic(&dirname) {
        iglob_inner(&dirname, recursive, true)
    } else {
        vec![dirname.clone()]
    };
    let mut out = vec![];
    for d in dirs {
        let names = if has_magic(&basename) {
            if recursive && isrecursive(&basename) {
                glob2(&d, dironly)
            } else {
                glob1(&d, &basename, dironly)
            }
        } else {
            glob0(&d, &basename)
        };
        for name in names {
            out.push(join(&d, &[&name]));
        }
    }
    out
}

/// glob.glob(pathname, recursive=...)
pub fn glob(pathname: &str, recursive: bool) -> Vec<String> {
    let mut out = iglob_inner(pathname, recursive, false);
    // iglob: skip the empty string yielded for a leading '**'
    if (pathname.is_empty() || (recursive && isrecursive(pathname.get(..2).unwrap_or(pathname))))
        && out.first().map(|s| s.is_empty()).unwrap_or(false)
    {
        out.remove(0);
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn paths_like_python() {
        assert_eq!(split("a/b/c"), ("a/b".into(), "c".into()));
        assert_eq!(split("/a"), ("/".into(), "a".into()));
        assert_eq!(split("//a"), ("//".into(), "a".into()));
        assert_eq!(split("a//b"), ("a".into(), "b".into()));
        assert_eq!(normpath("/a/./b/../c//d/"), "/a/c/d");
        assert_eq!(normpath("//a/.."), "//");
        assert_eq!(normpath("../a/../.."), "../..");
        assert_eq!(normpath(""), ".");
        assert_eq!(join("a", &["/b", "c"]), "/b/c");
        assert_eq!(join("a/", &["b"]), "a/b");
        assert_eq!(join("", &["b"]), "b");
    }

    #[test]
    fn fnmatch_like_python() {
        assert!(fnmatch("printer.cfg", "*.cfg"));
        assert!(fnmatch(".hidden.cfg", "*.cfg"));
        assert!(!fnmatch("a.cfg", "?.cfgx"));
        assert!(fnmatch("b1", "[a-c][!x]"));
        assert!(!fnmatch("bx", "[a-c][!x]"));
        assert!(fnmatch("[x", "[x"));
        assert!(fnmatch("a]", "[]a]]"));
        assert!(!fnmatch("b", "[z-a]"));
    }
}
