# Media Manifest

## Architecture and platform

| File | Bytes | SHA-256 | Provenance |
| --- | ---: | --- | --- |
| `go2_showcase.jpg` | `412790` | `a7b5a226e3e89d08aa04d932a4531dce7b2593e4a5d7e2693b5997f89652cd08` | Unitree Go2 reference platform supplied by the project owner on 2026-08-18 |
| `system_architecture.png` | `1150046` | `1dd30e72de523a4a14dc307dc7f0522687fa2c82f85ca9368873d9b9ce172298` | Project-owner architecture figure supplied on 2026-08-27 and checked against `ARCHITECTURE.md` |

## Reference engineering demo

These files are presentation derivatives of the two project-owner recordings
supplied on 2026-08-27. Their labels follow the image content rather than the
temporary upload filenames: the portrait Go2 recording is the third-person
view, while the desktop recording is the legacy first-person dashboard.

| File | Bytes | SHA-256 | Classification |
| --- | ---: | --- | --- |
| `demo/revisit_reference_third_view.mp4` | `3161544` | `6f91aa9b9f95fb47ebc1529a24e055d37b087463b5e9ba7290fa642502dd8819` | Browser H.264 third-person derivative |
| `demo/revisit_reference_third_view.gif` | `3994875` | `eb4238dd4f2920d6c4d85857dbb0274f5a6457fab1ce7e917ac3cd9cd1d913b5` | 48-frame, 8-second inline preview |
| `demo/revisit_reference_third_view_poster.jpg` | `35497` | `79b1b80d69173683bf34e2e2d12d0f567bf4bbf5cceade4b4c5921c02422f49a` | Mid-run poster |
| `demo/revisit_reference_dashboard.mp4` | `9922785` | `1aa1496da2517fcb2fe656c56a9097d2294948802d2fd1853ed0b8c10f40d7e0` | Browser H.264 legacy-dashboard derivative |
| `demo/revisit_reference_dashboard.gif` | `3844685` | `cec03c51334f2a397f3c78462e750c881ae27244a7425705658a5fadd9e67df9` | 48-frame, 8-second inline preview |
| `demo/revisit_reference_dashboard_poster.jpg` | `53351` | `a47ffdbbc34bd64899c5ab354995d0d7bb5295c6eed441a4c844e37e9789201d` | Mid-run poster |

The demo establishes presentation format only. It is not bound to a sealed
formal-run manifest and therefore is not evidence for SR/SPL or autonomous
arrival. Raw experiment evidence remains ignored under `runtime/`; publication
uses the workflow in `EXPERIMENT_DATA_COLLECTION.md`.

## Planned formal campaign namespace

The blank four-scene campaign in `REALWORLD_EVALUATION.md` reserves the
following browser-derivative names after independent review and manifest
finalization:

```text
scene01_formal_01_third_view.mp4
scene01_formal_01_third_view.gif
scene01_formal_01_dashboard.mp4
scene01_formal_01_dashboard.gif
scene01_formal_01_poster.jpg
```

Replace `scene01/formal_01` for Scene 01--04 and Formal 01--05. Source camera
masters remain outside Git; only H.264/GIF/poster derivatives may use this
namespace. A filename alone never establishes a formal result: every published
run must also link the finalized capture manifest, dataset manifest, independent
success/path record and SR/SPL calculation.
