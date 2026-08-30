//! The boundary's value shape: Python dicts, lists and scalars ↔
//! `serde_json::Value` ↔ the document types, so a `Joint`, a `Pose`, an
//! `InertialSpec` cross exactly as the `.riggen` file spells them
//! (docs/02-data-model.md §Schema) — with one difference: **ids are ints**.
//! The file writes `"l5"`; Python sees `5`. The keys that hold an id are
//! fixed by the schema (`id` a geom, `mesh` a mesh, `parent` / `child` a
//! link), so the rule is by key and nothing else is touched — a link
//! *named* `"l5"` lives under `name` and stays a string.

use pyo3::exceptions::{PyTypeError, PyValueError};
use pyo3::prelude::*;
use pyo3::types::{PyBool, PyDict, PyFloat, PyInt, PyList, PyString};
use serde::Serialize;
use serde::de::DeserializeOwned;
use serde_json::{Map, Number, Value};

/// Schema keys whose value is an id, with the letter the file prefixes it
/// with (`riggen_core::Id::PREFIX`).
const ID_KEYS: [(&str, char); 4] = [("id", 'g'), ("mesh", 'm'), ("parent", 'l'), ("child", 'l')];

/// A document value as Python, ids as ints.
pub fn to_doc<T: Serialize>(py: Python<'_>, value: &T) -> PyResult<Py<PyAny>> {
    let mut v = serde_json::to_value(value).map_err(|e| PyValueError::new_err(e.to_string()))?;
    ids_to_int(&mut v);
    to_py(py, &v)
}

/// A document value from Python; `what` names it in the error (`"joint:
/// missing field `axis`"`), which is `ValueError`.
pub fn from_doc<T: DeserializeOwned>(obj: &Bound<'_, PyAny>, what: &str) -> PyResult<T> {
    from_doc_with(obj, what, &[])
}

/// [`from_doc`] with `defaults` filled into a dict that lacks them —
/// `parent` / `child` for a joint the command overwrites anyway.
pub fn from_doc_with<T: DeserializeOwned>(
    obj: &Bound<'_, PyAny>,
    what: &str,
    defaults: &[(&str, Value)],
) -> PyResult<T> {
    let mut v = from_py(obj)?;
    if let Value::Object(map) = &mut v {
        for (key, value) in defaults {
            map.entry(*key).or_insert_with(|| value.clone());
        }
    }
    ids_to_str(&mut v);
    serde_json::from_value(v).map_err(|e| PyValueError::new_err(format!("{what}: {e}")))
}

fn ids_to_int(v: &mut Value) {
    match v {
        Value::Object(map) => {
            for (key, value) in map.iter_mut() {
                if ID_KEYS.iter().any(|(k, _)| k == key)
                    && let Value::String(s) = value
                    && let Some(n) = parse_id(s)
                {
                    *value = Value::Number(n.into());
                    continue;
                }
                ids_to_int(value);
            }
        }
        Value::Array(items) => items.iter_mut().for_each(ids_to_int),
        _ => {}
    }
}

fn ids_to_str(v: &mut Value) {
    match v {
        Value::Object(map) => {
            for (key, value) in map.iter_mut() {
                if let Some((_, prefix)) = ID_KEYS.iter().find(|(k, _)| k == key)
                    && let Value::Number(n) = value
                    && let Some(n) = n.as_u64()
                {
                    *value = Value::String(format!("{prefix}{n}"));
                    continue;
                }
                ids_to_str(value);
            }
        }
        Value::Array(items) => items.iter_mut().for_each(ids_to_str),
        _ => {}
    }
}

/// `"l5"` → `5`; anything else is not an id.
fn parse_id(s: &str) -> Option<u32> {
    let mut chars = s.chars();
    let prefix = chars.next()?;
    let digits = chars.as_str();
    if !prefix.is_ascii_lowercase()
        || digits.is_empty()
        || !digits.bytes().all(|b| b.is_ascii_digit())
    {
        return None;
    }
    digits.parse().ok()
}

fn to_py(py: Python<'_>, v: &Value) -> PyResult<Py<PyAny>> {
    Ok(match v {
        Value::Null => py.None(),
        Value::Bool(b) => PyBool::new(py, *b).to_owned().into_any().unbind(),
        Value::Number(n) => {
            if let Some(u) = n.as_u64() {
                u.into_pyobject(py)?.into_any().unbind()
            } else if let Some(i) = n.as_i64() {
                i.into_pyobject(py)?.into_any().unbind()
            } else {
                n.as_f64()
                    .unwrap_or(f64::NAN)
                    .into_pyobject(py)?
                    .into_any()
                    .unbind()
            }
        }
        Value::String(s) => PyString::new(py, s).into_any().unbind(),
        Value::Array(items) => {
            let items = items
                .iter()
                .map(|item| to_py(py, item))
                .collect::<PyResult<Vec<_>>>()?;
            PyList::new(py, items)?.into_any().unbind()
        }
        Value::Object(map) => {
            let dict = PyDict::new(py);
            for (key, value) in map {
                dict.set_item(key, to_py(py, value)?)?;
            }
            dict.into_any().unbind()
        }
    })
}

fn from_py(obj: &Bound<'_, PyAny>) -> PyResult<Value> {
    if obj.is_none() {
        return Ok(Value::Null);
    }
    if obj.is_instance_of::<PyBool>() {
        return Ok(Value::Bool(obj.extract()?));
    }
    if obj.is_instance_of::<PyInt>() {
        return Ok(Value::Number(if let Ok(u) = obj.extract::<u64>() {
            u.into()
        } else {
            obj.extract::<i64>()?.into()
        }));
    }
    if obj.is_instance_of::<PyFloat>() {
        let f: f64 = obj.extract()?;
        return Number::from_f64(f)
            .map(Value::Number)
            .ok_or_else(|| PyValueError::new_err(format!("{f} is not a finite number")));
    }
    if obj.is_instance_of::<PyString>() {
        return Ok(Value::String(obj.extract()?));
    }
    if let Ok(dict) = obj.cast::<PyDict>() {
        let mut map = Map::new();
        for (key, value) in dict.iter() {
            let key: String = key
                .extract()
                .map_err(|_| PyTypeError::new_err(format!("dict key {key} is not a str")))?;
            map.insert(key, from_py(&value)?);
        }
        return Ok(Value::Object(map));
    }
    if let Ok(iter) = obj.try_iter() {
        return iter
            .map(|item| from_py(&item?))
            .collect::<PyResult<Vec<_>>>()
            .map(Value::Array);
    }
    Err(PyTypeError::new_err(format!(
        "cannot use {} as a document value (expected None, bool, int, float, str, dict, or a sequence)",
        obj.get_type().name()?
    )))
}
