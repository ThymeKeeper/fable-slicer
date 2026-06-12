//! Program state that persists between runs — the conveniences, never the
//! physics: which profiles were selected, where the file dialogs left off.
//! Lives in the app's dotfile folder beside the user profiles
//! (`<config>/fable-slicer/state.toml`). Best-effort at both ends: a missing
//! or unreadable file is just defaults, and a failed save must never block
//! the app.

use serde::{Deserialize, Serialize};
use std::path::{Path, PathBuf};

/// Fable Slicer's per-user dotfile folder (`~/.config/fable-slicer` on
/// Linux, `%APPDATA%\fable-slicer` on Windows, `~/Library/Application
/// Support/fable-slicer` on macOS): user profiles live under `profiles/`,
/// program state in `state.toml`. Adopts the pre-rename `slicer` folder the
/// first time it's missing — moved wholesale, profiles and state together.
pub fn config_dir() -> Option<PathBuf> {
    let base = if cfg!(target_os = "windows") {
        std::env::var_os("APPDATA").map(PathBuf::from)
    } else if cfg!(target_os = "macos") {
        std::env::var_os("HOME").map(|h| PathBuf::from(h).join("Library/Application Support"))
    } else {
        std::env::var_os("XDG_CONFIG_HOME")
            .map(PathBuf::from)
            .or_else(|| std::env::var_os("HOME").map(|h| PathBuf::from(h).join(".config")))
    }?;
    let dir = base.join("fable-slicer");
    adopt_legacy_dir(&base.join("slicer"), &dir);
    Some(dir)
}

/// One-time rename migration: move the old dotfile folder to the new name.
/// Best-effort — failure just means starting with a fresh folder, the old
/// data stays where it was.
fn adopt_legacy_dir(old: &Path, new: &Path) {
    if !new.exists() && old.is_dir() {
        let _ = std::fs::rename(old, new);
    }
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct AppState {
    /// Selected profile names. Empty = never saved; names that no longer
    /// resolve (a deleted user profile) fall back tier by tier on load.
    pub printer: String,
    pub filament: String,
    pub process: String,
    /// Where the STL import dialog last picked a file.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub last_model_dir: Option<PathBuf>,
    /// Where the g-code export dialog last saved.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub last_export_dir: Option<PathBuf>,
}

impl AppState {
    /// `<config>/fable-slicer/state.toml`.
    pub fn path() -> Option<PathBuf> {
        config_dir().map(|d| d.join("state.toml"))
    }

    /// The saved state, or defaults — never an error.
    pub fn load() -> Self {
        Self::path().map(|p| Self::load_from(&p)).unwrap_or_default()
    }

    fn load_from(path: &Path) -> Self {
        std::fs::read_to_string(path)
            .ok()
            .and_then(|t| toml::from_str(&t).ok())
            .unwrap_or_default()
    }

    pub fn save(&self) -> Result<(), String> {
        let path = Self::path().ok_or("no user config directory available")?;
        self.save_to(&path)
    }

    fn save_to(&self, path: &Path) -> Result<(), String> {
        if let Some(dir) = path.parent() {
            std::fs::create_dir_all(dir).map_err(|e| e.to_string())?;
        }
        let text = toml::to_string_pretty(self).map_err(|e| e.to_string())?;
        std::fs::write(path, text).map_err(|e| e.to_string())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn state_roundtrips() {
        let dir = std::env::temp_dir().join(format!("slicer-state-{}", std::process::id()));
        let path = dir.join("state.toml");
        let s = AppState {
            printer: "sovol-zero-custom".into(),
            filament: "pla".into(),
            process: "sovol-zero-custom".into(),
            last_model_dir: Some("/tmp/models".into()),
            last_export_dir: None,
        };
        s.save_to(&path).unwrap();
        assert_eq!(AppState::load_from(&path), s);
        std::fs::remove_dir_all(&dir).ok();
    }

    #[test]
    fn legacy_dotfile_folder_is_adopted_once() {
        let base = std::env::temp_dir().join(format!("fable-migrate-{}", std::process::id()));
        let (old, new) = (base.join("slicer"), base.join("fable-slicer"));
        std::fs::create_dir_all(old.join("profiles")).unwrap();
        std::fs::write(old.join("state.toml"), "printer = \"x\"\n").unwrap();
        adopt_legacy_dir(&old, &new);
        assert!(!old.exists(), "old folder moved away");
        assert!(new.join("profiles").is_dir(), "profiles came along");
        assert_eq!(AppState::load_from(&new.join("state.toml")).printer, "x");
        // A second old folder appearing later never clobbers the new one.
        std::fs::create_dir_all(&old).unwrap();
        adopt_legacy_dir(&old, &new);
        assert!(old.exists(), "existing new folder wins");
        std::fs::remove_dir_all(&base).ok();
    }

    #[test]
    fn missing_or_garbage_state_is_just_defaults() {
        let dir = std::env::temp_dir().join(format!("slicer-state-bad-{}", std::process::id()));
        let path = dir.join("state.toml");
        assert_eq!(AppState::load_from(&path), AppState::default(), "missing file");
        std::fs::create_dir_all(&dir).unwrap();
        std::fs::write(&path, "not = [valid").unwrap();
        assert_eq!(AppState::load_from(&path), AppState::default(), "garbage file");
        // Unknown keys from a future version load fine (forward compatible).
        std::fs::write(&path, "printer = \"x\"\nfuture_key = 3\n").unwrap();
        assert_eq!(AppState::load_from(&path).printer, "x");
        std::fs::remove_dir_all(&dir).ok();
    }
}
