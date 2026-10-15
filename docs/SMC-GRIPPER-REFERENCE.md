# SMC MHZL2 16D2 gripper reference

Updated October 14, 2026. The user confirms the physical gripper is SMC MHZL2-16D2. This resolves the earlier website SCHUNK/starter SMC mismatch; treat the website label as outdated for our cell. Mounted custom fingers, actual pressure, TCP validation and available feedback still need cell-level confirmation.

## Official resources and verified facts

[Exact SMC product page](https://www.smcworld.com/webcatalog/s3s/en-jp/detail/?partNumber=MHZL2-16D2): parallel long-stroke air gripper; 16 mm bore; D means double acting; suffix 2 means through-holes in opening/closing direction. It is a finger mounting option, not a doubled stroke. The exact bare part number has no auto-switch suffix; do not assume CSI has no separately installed sensors. PDF and 2D/3D CAD services require registered login on this page.

[SMC MHZ catalog](https://content2.smcetech.com/pdf/MHZ_EU.pdf), PDF pages 30–31 (zero-based 29–30), ordering/specifications: MHZL2-16D total opening/closing stroke is 12 mm, explicitly BOTH sides combined. Symmetric travel corresponds to 6 mm per jaw, consistent with the starter. Repeatability ±0.01 mm is an actuator catalog value, not overall vision/robot placement accuracy. Operating pressure range for the 16 mm double-acting model is 0.1–0.7 MPa; this is not authorization to alter CSI pressure. Force depends on pressure and gripping-point distance; use the force/offset graphs and attachment limits for the actual finger design, not a headline force value as guaranteed payload.

[SMC USA series page](https://www.smcusa.com/products/mhzl2-air-gripper-parallel-type-long-stroke~20299): series overview and CAD/configuration access.

## Project implications

Keep existing fingers for the 10 mm cube baseline. Stroke is travel, NOT usable tip opening. The starter's approximately 4–16 mm hooked-tip gap comes from custom profile geometry and is not a universal SMC specification. Vendor CAD describes the actuator/native fingers, not CSI's elongated custom attachments. Reuse supplied custom geometry, inspect actual mounting/contact depth/open/closed gaps, and validate grasping on the cell.

Only redesign if a selected manufacturing part cannot be grasped reliably by the existing fingers. Check attachment dimensions, contact point/lever arm, load/moment and interference before proposing CAD. Do not assume double acting means programmable force or continuous position control; actual valve/I/O and feedback interface remain to confirm. Printed attachments still require CSI review.

Replace the broad gripper identity question with: can we validate existing finger contact geometry/gap/TCP for 10 mm cubes, what regulated pressure is used, and what open/closed or part-presence feedback is exposed through the approved interface? No need to ask which actuator is installed anymore.
