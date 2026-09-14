# Core V5.5 LOOP APPLY SIGNFIX

This build is generated from the last AFTER frame:

- Current pixel = (1101.8153, 1142.3672)
- Target pixel  = (1060.0000, 943.0000)
- Current error = 203.7052 px

The previous APPLY experiment proved that `Motion_StartRelative()` has the
opposite sign convention from the affine machine-coordinate direction.

Affine differential request:
- dX_affine = -39704.773
- dY_affine = -8843.654

Therefore the relative motor command is sign-inverted:
- X command = +39705 counts
- Y command = +8844 counts

## Critical rule

Do NOT Home.
Do NOT power-cycle the MKS motor drivers.
Do NOT manually move the stage before pressing KEY0.

STM32 may be reflashed/reset.

## Procedure

1. Keep both MKS drivers powered.
2. Build/download this firmware and exit Keil Debug.
3. Start `GrabImage_1x_LOOP_AFTER_SIGNFIX.py`.
4. Do NOT Home.
5. Press KEY0 once.
6. Firmware executes:
   - X relative +39705 counts
   - Y relative +8844 counts
   - settle 500 ms
   - one PB0 trigger
7. Python reports the new residual.
