//! Program state that persists between runs — the conveniences, never the
//! physics: which profiles were selected, where the file dialogs left off.
//! Lives in the app's dotfile folder beside the user profiles
//! (`<config>/fable-slicer/state.toml`). Best-effort at both ends: a missing
//! or unreadable file is just defaults, and a failed save must never block
//! the app.

use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;
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

/// A user-defined pseudo color: a named blend of tool slots, realized at
/// slice time by layer dithering. Weights are relative shares (they needn't
/// sum to anything); slots reference the `tools` list by index.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct BlendState {
    pub name: String,
    pub weights: Vec<(u32, f32)>,
    /// The spools this blend draws from — its own sub-palette. The editor's
    /// mix surface follows this count (two → ramp, three → triangle, more →
    /// chip lattice). Empty = every loaded slot.
    pub tools: Vec<u32>,
}

/// What one machine currently holds: the filament loaded in each tool slot,
/// the spool colors over those slots, the blend palette mixed from them, and
/// the process last used on it. All of it describes a physical setup —
/// machines don't share spools — and blends reference slots by index, so the
/// whole bundle only means anything relative to one machine. Keyed by
/// printer profile name in [`AppState::loadouts`].
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct Loadout {
    /// Filament profile name per tool slot; slot 0 is the filament tier.
    pub tools: Vec<String>,
    /// Loaded-spool color per slot ("#RRGGBB", "" = none): an override of the
    /// slot filament's profile color, describing the spool physically loaded.
    /// Aligned with `tools` — a slot whose filament changes drops its
    /// override.
    pub tool_colors: Vec<String>,
    /// The blend palette — pseudo colors dithered from the tool slots.
    pub blends: Vec<BlendState>,
    /// The process last used on this machine ("" = never recorded).
    pub process: String,
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct AppState {
    /// Selected profile names. Empty = never saved; names that no longer
    /// resolve (a deleted user profile) fall back tier by tier on load.
    pub printer: String,
    pub filament: String,
    pub process: String,
    /// Filament profile name per tool slot on a toolchanger. `filament`
    /// stays the tool-0 legacy mirror: an old state without `tools` reads as
    /// one slot (see [`Self::tool_filaments`]); saves write both.
    ///
    /// `tools`, `tool_colors`, `blends`, and `process` are the ACTIVE
    /// printer's loadout mirrored flat — the legacy read path and what old
    /// versions understand. The per-machine truth lives in [`Self::loadouts`].
    pub tools: Vec<String>,
    /// Loaded-spool color per slot ("#RRGGBB", "" = none): an override of the
    /// slot filament's profile color, describing the spool physically loaded.
    /// Aligned with `tools` as written — a slot whose filament changes drops
    /// its override.
    pub tool_colors: Vec<String>,
    /// The blend palette — pseudo colors dithered from the tool slots.
    pub blends: Vec<BlendState>,
    /// Each machine's own loadout, keyed by printer profile name: switching
    /// printers swaps loadouts instead of dragging one machine's spools onto
    /// another. Loading folds legacy flat fields in as the then-active
    /// printer's entry (see [`Self::adopt_flat_loadout`]).
    #[serde(skip_serializing_if = "BTreeMap::is_empty")]
    pub loadouts: BTreeMap<String, Loadout>,
    /// Where the STL import dialog last picked a file.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub last_model_dir: Option<PathBuf>,
    /// Where the g-code export dialog last saved.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub last_export_dir: Option<PathBuf>,
    /// The GUI's 3D-view accent color ("#RRGGBB") — the one hue its model
    /// tint, feature palette, and heat ramps are derived from.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub accent: Option<String>,
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
        let mut s: Self = std::fs::read_to_string(path)
            .ok()
            .and_then(|t| toml::from_str(&t).ok())
            .unwrap_or_default();
        s.adopt_flat_loadout();
        s
    }

    /// Fold the flat loadout fields into the per-printer map when it lacks
    /// an entry for the saved printer — a legacy state's flat fields describe
    /// the machine that was active when it was written. States saved by this
    /// version already carry the entry (the flat fields are its mirror), so
    /// this only fires on old files.
    fn adopt_flat_loadout(&mut self) {
        if !self.printer.is_empty() && !self.loadouts.contains_key(&self.printer) {
            self.loadouts.insert(
                self.printer.clone(),
                Loadout {
                    tools: self.tool_filaments(),
                    tool_colors: self.tool_colors.clone(),
                    blends: self.blends.clone(),
                    process: self.process.clone(),
                },
            );
        }
    }

    pub fn save(&self) -> Result<(), String> {
        let path = Self::path().ok_or("no user config directory available")?;
        self.save_to(&path)
    }

    fn save_to(&self, path: &Path) -> Result<(), String> {
        if let Some(dir) = path.parent() {
            std::fs::create_dir_all(dir).map_err(|e| e.to_string())?;
        }
        // Write both spellings: `tools` (one name per slot) and the legacy
        // `filament` as the tool-0 mirror.
        let mut st = self.clone();
        st.tools = st.tool_filaments();
        st.filament = st.tools[0].clone();
        let text = toml::to_string_pretty(&st).map_err(|e| e.to_string())?;
        std::fs::write(path, text).map_err(|e| e.to_string())
    }

    /// The filament selection per tool slot: `tools`, or — legacy states
    /// saved before toolchangers — the single `filament` as slot 0.
    pub fn tool_filaments(&self) -> Vec<String> {
        if self.tools.is_empty() {
            vec![self.filament.clone()]
        } else {
            self.tools.clone()
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn state_roundtrips() {
        let dir = std::env::temp_dir().join(format!("slicer-state-{}", std::process::id()));
        let path = dir.join("state.toml");
        let sovol = Loadout {
            tools: vec!["pla".into(), "petg".into(), "asa".into()],
            tool_colors: vec!["#F2F2F2".into(), String::new(), "#1A1A1A".into()],
            blends: vec![BlendState {
                name: "25% grey".into(),
                weights: vec![(0, 3.0), (2, 1.0)],
                tools: vec![0, 2],
            }],
            process: "sovol-zero-custom".into(),
        };
        let voron = Loadout {
            tools: vec!["asa".into(); 5],
            tool_colors: vec!["#0000FF".into()],
            blends: Vec::new(),
            process: "standard".into(),
        };
        let s = AppState {
            printer: "sovol-zero-custom".into(),
            filament: "pla".into(),
            process: "sovol-zero-custom".into(),
            tools: sovol.tools.clone(),
            tool_colors: sovol.tool_colors.clone(),
            blends: sovol.blends.clone(),
            loadouts: [
                ("sovol-zero-custom".to_string(), sovol),
                ("voron 2.4 (garage)".to_string(), voron),
            ]
            .into_iter()
            .collect(),
            last_model_dir: Some("/tmp/models".into()),
            last_export_dir: None,
            accent: Some("#D8A852".into()),
        };
        s.save_to(&path).unwrap();
        assert_eq!(AppState::load_from(&path), s);
        std::fs::remove_dir_all(&dir).ok();
    }

    #[test]
    fn legacy_flat_fields_seed_the_active_printers_loadout() {
        let dir = std::env::temp_dir().join(format!("slicer-state-legacy-{}", std::process::id()));
        let path = dir.join("state.toml");
        std::fs::create_dir_all(&dir).unwrap();
        // A pre-loadout file: flat fields only. Loading folds them into the
        // then-active printer's map entry.
        std::fs::write(
            &path,
            r##"
printer = "voron24"
filament = "pla"
process = "standard"
tools = ["pla", "petg"]
tool_colors = ["#FFFFFF", ""]
[[blends]]
name = "grey"
weights = [[0, 1.0], [1, 1.0]]
"##,
        )
        .unwrap();
        let s = AppState::load_from(&path);
        let lo = s.loadouts.get("voron24").expect("legacy flat fields must seed a loadout");
        assert_eq!(lo.tools, vec!["pla".to_string(), "petg".to_string()]);
        assert_eq!(lo.tool_colors, vec!["#FFFFFF".to_string(), String::new()]);
        assert_eq!(lo.blends.len(), 1);
        assert_eq!(lo.process, "standard");
        // A file that already carries a loadout for the printer is left
        // alone — the map entry wins over the flat mirror.
        std::fs::write(
            &path,
            r#"
printer = "voron24"
process = "standard"
tools = ["pla"]
[loadouts.voron24]
tools = ["asa"]
process = "draft"
"#,
        )
        .unwrap();
        let s = AppState::load_from(&path);
        assert_eq!(s.loadouts["voron24"].tools, vec!["asa".to_string()]);
        assert_eq!(s.loadouts["voron24"].process, "draft");
        // A never-saved state (no printer) seeds nothing.
        let fresh = AppState::default();
        let mut fresh2 = fresh.clone();
        fresh2.adopt_flat_loadout();
        assert_eq!(fresh, fresh2);
        std::fs::remove_dir_all(&dir).ok();
    }

    #[test]
    fn filament_stays_the_tool_zero_mirror() {
        let dir = std::env::temp_dir().join(format!("slicer-state-tools-{}", std::process::id()));
        let path = dir.join("state.toml");
        // A legacy state (no `tools`) reads as a single slot...
        let legacy = AppState { filament: "pla".into(), ..Default::default() };
        assert_eq!(legacy.tool_filaments(), vec!["pla".to_string()]);
        // ...and saving writes both spellings.
        legacy.save_to(&path).unwrap();
        let loaded = AppState::load_from(&path);
        assert_eq!(loaded.tools, vec!["pla".to_string()]);
        assert_eq!(loaded.filament, "pla");
        // A save with slots keeps `filament` synced to tools[0] on disk.
        let s = AppState { filament: "stale".into(), tools: vec!["asa".into(), "petg".into()], ..Default::default() };
        s.save_to(&path).unwrap();
        let loaded = AppState::load_from(&path);
        assert_eq!(loaded.filament, "asa");
        assert_eq!(loaded.tools, vec!["asa".to_string(), "petg".to_string()]);
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
