# Tier 0 results

Stock-camera checks (plans/RECORDINGS_NEEDED.md §1). Raw files and figures stay in `recordings/tier0/<place>/` (gitignored). Script: `PC-part/proto/scripts/tier0_check.py`.

## Place 1: main desk (2026-09-28)

**Scene:** white textured-plaster ceiling with a smoke detector, plus nearby objects at other depths: tall shelving, monitor, an overhead wooden panel, and (ultra-wide) the user's head.

**Captures:** photo main 1× (5.4 mm, 1/100 s, ISO 250), photo ultra-wide 0.6× (2.2 mm, 1/50 s, ISO 250), video 8K (4320×7680) at 30 fps, 23 s, stock auto exposure, stabilisation state unknown.

### Features per frame (Shi-Tomasi, quality 0.01, min dist 12 px, max 400)

| Camera @ width | raw | with CLAHE | 4×4 grid cells covered (raw) |
|---|---|---|---|
| main @ 640 | 86 | 400 (cap) | 12/16 |
| main @ 1280 | 400 (cap) | 400 | 14/16 |
| ultra-wide @ 640 | 168 | 363 | 15/16 |
| ultra-wide @ 1280 | 400 (cap) | 400 | 16/16 |

The plaster texture is real and CLAHE brings it out strongly. Ultra-wide at 640 already gives ~2× the raw features of main.

### Video tracking (360×640, LK + forward-backward, rotation+translation RANSAC at 0.5 px)

| | frames | tracks (median) | inliers of one rigid motion | sharpness |
|---|---|---|---|---|
| phone still | 59 | 53 | **100 %** | 165 |
| phone moving | 622 | 52 | **45 %** | 54 |

### What it means
1. **Texture: OK.** Enough trackable points at 640 px, more with CLAHE or the ultra-wide. This ceiling is not a blocker.
2. **Multiple depths: a real problem for the current design.** While sliding, flow magnitudes differ by ~3× across the image (nearby shelving/monitor/panel vs ceiling), so only ~45 % of points fit one motion. DESIGN.md §6.2–6.3 assume a single plane (one ρ). At a desk with shelves, a monitor and a hutch this is the normal case, not an edge case. → Design change in notes/DECISIONS.md.
3. **Motion blur: stock-video artefact.** Sharpness drops 3× while moving because the stock app used ~1/30 s exposure. The recorder's ≤ 4 ms manual exposure is meant to fix this; to be verified in Tier 1.
4. **Moving things in view** (your head, the monitor's picture) must be rejected as outliers; FB check + RANSAC already handle most of it, and a per-track consistency check will cover the rest.

Caveats: stock camera, unknown EIS, 30 fps with long exposure. Numbers are indicative; the recorder data in Tier 1 is what counts.

Place facts: ceiling **1.90 m** above the desk; desk covered by a **very large mouse pad** (soft surface: taps will be damped, sliding friction is cloth-like); the overhead wooden panel is a **shelf above the desk** (a near plane at a few tens of cm, partly in view).
