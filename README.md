# HkxHbSharp

Reader and solver for TotK Phive helper bones (`.bphhb`).

A `.bphhb` is an AAMP parameter archive (not a Havok tagfile) that describes a procedural bone rig: driver bones watch
a few skeleton bones, Hermite curves turn their swing and twist into numbers, and pose-driven transforms turn those
back into the transforms of secondary bones (shoulder pads, skirt anchors, twist and corrective joints).

- `BphhbReader` - reads the archive with AampSharp into `HelperBoneData`.
- `HelperBoneData` - the rig as authored; every `BoneId` indexes `Bones`.
- `HelperBoneSolver` - `Solve(Matrix4x4[] worldTransforms)` overwrites the driven bones in place.

The solver is ported from Marrow and is **not verified against the game**.
