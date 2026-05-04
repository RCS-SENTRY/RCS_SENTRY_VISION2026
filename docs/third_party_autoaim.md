# Third-party Autoaim Notes

`rm_autoaim` V3 references and adapts ideas from `CSU-FYT-Vision/FYT2024_vision`.

Used ideas:

- IPPE/PNP multi-solution selection and reprojection-error sanity checks.
- Target selection that prefers a stable tracked armor and falls back to image-center armor selection.
- Debug-friendly tracker states: `LOST`, `DETECTING`, `TRACKING`, `TEMP_LOST`.
- Manual distance-based yaw/pitch compensation.

Not imported:

- FYT bringup, launch, camera driver, serial driver, navigation, decision, messages, or calibration files.
- FYT hard-coded camera intrinsics/extrinsics.

License:

- FYT source files used as references carry Apache-2.0 headers, with some tracker lineage noting MIT-origin code plus Apache-2.0 modifications. The RCS V3 adapter files retain Apache-2.0 attribution where FYT logic was adapted.

