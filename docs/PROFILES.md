# Profiles: tiers, inheritance, and precedence

Settings come from three **tiers**, each answering a different question:

| tier | answers | owns (examples) |
|---|---|---|
| **printer** | what machine? | bed size, nozzle Ø, accel/jerk, retraction, z-hop, travel/print/first-layer speed, start/end g-code |
| **filament** | what material? | diameter, density, temperatures, flow multiplier, max volumetric speed, pressure advance, fan/cooling |
| **process** | what quality? | layer height, walls, top/bottom, infill (density/pattern/overlap), supports, seams, fuzzy/ironing/vase, per-feature speeds |

You always slice with one profile *selected per tier* — `(printer, filament,
process)` — and the three are flattened into the engine's `Settings`.

## Who overrides whom

**Across tiers there is almost nothing to override.** Each setting belongs to
exactly one tier (temperatures are filament-only, wall count is process-only,
bed size is printer-only), so a filament profile and a process profile can
never fight: both apply in full. The deliberate exceptions:

- **`print_speed` and `first_layer_speed`** exist in both the printer and
  process tiers; **printer wins** (it's a machine capability — a `draft`
  process shouldn't outrun a Mini, and a Voron shouldn't crawl because the
  process was written for an Ender). The GUI therefore routes edits of these
  to the printer tier when saving.
- **Derived per-feature speeds**: if a process profile doesn't pin
  `external_perimeter`/`solid`/`support`/`gap_fill` speed, they're computed
  from the winning print speed (50% / 80% / 90% / 40 %≤40) at resolve time.

So the full resolution order for any one field is:

```
code default  →  owning tier's inherits chain (parent → child)  →  done
```

with the two shared speed fields resolving `printer → process → default`, and
panel edits in the GUI sitting on top of everything until you save or switch.

## Inheritance (within a tier)

A profile may name a parent: `inherits = "petg"`. Resolution walks the chain
root-first and the **child overrides the parent field-by-field**; unset fields
fall through. Chains can be any depth; cycles are an error.

## Built-in vs. user profiles

- **Built-ins** (`generic`/`voron24`/`sovol-zero`, `pla`/`petg`,
  `draft`/`standard`/`fine`) are embedded in the binary and **read-only** —
  they can't be overwritten, deleted, or shadowed from the user directory.
  A user-dir file named like a built-in is skipped with a warning; base
  yours on it with `inherits` instead.
- **User profiles** live in the platform config dir and are auto-loaded by
  both the GUI and the CLI:
  - Linux: `~/.config/slicer/profiles/{printer,filament,process}/*.toml`
  - macOS: `~/Library/Application Support/slicer/profiles/…`
  - Windows: `%APPDATA%\slicer\profiles\…`

  Saved files are **minimal diffs**: only the fields you changed, plus
  `inherits = "<what you based it on>"`. Fix the parent and every child
  benefits.
- `--profile-dir <dir>` (CLI) layers an extra directory on top and *is*
  allowed to shadow anything — an explicit power feature for experiments.

## In the GUI

Each tier row shows `*` when your panel edits differ from the selected
profile, with the changed fields routed to their owning tier automatically:

- **💾 Save** — new name ⇒ a diff inheriting the current selection;
  same name (user profile) ⇒ the new diff merges over the stored fields and
  keeps its original parent. Saving one tier leaves your unsaved edits in the
  other tiers untouched.
- **🗑 Delete** — user profiles only, with confirmation; the selection falls
  back to a built-in.

Example: select `petg` + `standard`, raise nozzle temp to 245 and walls to 4,
then save the filament tier as `my-petg` and the process tier as `strong`:

```toml
# ~/.config/slicer/profiles/filament/my-petg.toml
inherits = "petg"
nozzle_temp_c = 245

# ~/.config/slicer/profiles/process/strong.toml
inherits = "standard"
wall_count = 4
```

`slicer-cli model.stl --filament my-petg --process strong` now uses both —
and a future tweak to the built-in `petg` (or `standard`) flows through.
