# Core V5.5 CAL 3X3 SAFE

This build is a minimal patch on top of the already tested
`Core_V5_5_CAL_3X3` build.

## Build marker

```c
g_build_id = 0x2609133AU;
```

## Why the grid changed

The previous run showed the entire `X=79920` column was visually abnormal,
while the reliable portion of the run was concentrated in the
`X=96304 .. 112688` region.  The SAFE grid therefore avoids X=79920.

## SAFE 3x3 machine coordinates

- X = 96304 / 104496 / 112688
- Y = 20480 / 36864 / 53248
- X spacing = 8192 counts
- Y spacing = 16384 counts

Capture order is a continuous serpentine path:

| Capture | Point | Machine X | Machine Y |
|---:|---|---:|---:|
| 1 | P0 | 96304 | 20480 |
| 2 | P1 | 104496 | 20480 |
| 3 | P2 | 112688 | 20480 |
| 4 | P3 | 112688 | 36864 |
| 5 | P4 | 104496 | 36864 |
| 6 | P5 | 96304 | 36864 |
| 7 | P6 | 96304 | 53248 |
| 8 | P7 | 104496 | 53248 |
| 9 | P8 | 112688 | 53248 |

Physical path:

```text
P6 ---- P7 ---- P8
|               |
P5 ---- P4 ---- P3
|               |
P0 ---- P1 ---- P2
```

## Run procedure

1. X/Y driver `En` should both be `Hold`.
2. Rebuild/download this SAFE project in Keil, then exit Debug.
3. Run `GrabImage_9x_CAL_3X3_SAFE.py`.
4. Reset STM32 once.
5. Press KEY0 once: one Home sequence only.
6. P0 is captured automatically after the first safe calibration position.
7. After each Python frame appears, press KEY0 once for the next point.
8. Do not Reset or re-Home between P0 and P8.
9. After P8, Python auto-stops and writes CSV + summary + preliminary affine fit.

The Python logger independently marks suspicious frames as
`calibration_valid=0`; those rows are not used in the preliminary affine fit.
