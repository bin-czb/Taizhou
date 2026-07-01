# Calibration Target Printing Notes

Print these PDFs with **Actual size / 100% scale**.

Do not use:

- Fit to page
- Shrink oversized pages
- Borderless auto scaling
- Any non-uniform stretch

## AprilTag Square Check

Recommended files:

- `apriltag_print/square_verified/tagStandard41h12_id0_A4_outer100mm_square_check.pdf`
- `apriltag_print/square_verified/tagStandard41h12_ids_0_3_A4_outer50mm_square.pdf`

After printing, measure the printed tag outer black-square width and height.
They must match. If width and height differ, the print is not usable for accurate pose estimation.

Use the measured value, in meters, as the tag size.

## Checkerboard For Camera Intrinsics

Recommended file:

- `checkerboard_print/checkerboard_A4_9x6_inner_corners_20mm.pdf`

Generated pattern:

- inner corners: `9 x 6`
- nominal square size: `20 mm = 0.020 m`

After printing, measure several squares with a ruler or caliper and use the actual square size in calibration.
