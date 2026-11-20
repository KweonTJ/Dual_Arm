# Centered gripper CAD source

User-provided ORV_Centered_Gripper_Inventor, dated 2026-11-18.
Imported into the dual-arm URDF on 2026-11-21. Source directory was read only.

- Parts/, Reference/: STEP inputs used to tessellate the actual geometry.
- InventorData/Placements.csv: independent closed/open assembly transforms in mm.
- Source/CAD/: supplied regeneration sources and original jaw inputs.
- Source/Sources.txt: attribution and provenance.
- Source/OpenArm_1.0.1/LICENSE.txt: retained CERN-OHL-S-2.0 text.
- source_manifest.json: SHA-256 hashes of retained inputs.

The centered adapters and jaw derivatives carry the supplied CERN-OHL-S-2.0
notice; original OpenArm design is Copyright 2025 Enactic, Inc., source commit
0403835afae64949ede85e440e86a283b747b9dd. Reference purchase components retain
their original notices. See Source/Sources.txt for the detailed scope.

URDF integration adds metre-scale tessellation, rigid grouping, passive joint
frames, and closed-loop simulation constraints. Regeneration scripts are under
src/dual_arm_description/scripts/. No STEP solids were modified.

Mass assumptions are provisional: printed C02/C03/C05/C06/F01/F02 parts use
1240 kg/m3; remaining metal/catalog envelopes use 7800 kg/m3; the AX-12A case
uses 0.0546 kg. This estimates a metal design assembly, not the mass of the white
printed prototype in the photos. Box inertias approximate each rigid group.
