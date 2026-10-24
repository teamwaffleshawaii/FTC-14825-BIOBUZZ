# BIOBUZZ — First Tournament Software Checklist

Reviewed: 2026-10-23. This is a source review, **not** a successful build or field certification.

## OpModes to confirm on the Driver Station
- TeleOp RED / TeleOp BLUE: normal driver controls and alliance-specific flywheel AprilTags.
- TeleOpTurret-RED / TeleOpTurret-BLUE: same controls plus AprilTag turret auto-aim.
- Red Left / Red Right / Blue Left / Blue Right: programmed autonomous routes and vision velocity sampling before each volley.
- Park Only: backup autonomous. Verify movement direction and starting orientation.

## Before entering the field
- [ ] Pull latest `main`, perform a clean Android Studio build, install on the Robot Controller, and verify every intended OpMode appears.
- [ ] Confirm correct alliance OpMode and correct AprilTag IDs (RED 30–37; BLUE 38–45).
- [ ] Inspect all wiring, 6V servo supply, servo gear mesh, fasteners, battery charge, and mechanism clearance.
- [ ] Verify AXON MAX+ turret moves freely. The software permits servo positions 0.00–1.00; these are **not verified mechanical limits**.
- [ ] Check that turret initialization to 0.46 cannot strike a hard stop.
- [ ] Confirm Limelight pipeline 0 is AprilTag, camera has current frames, and tags are visible from both scoring positions.
- [ ] Test field-centric driving, heading reset (Gamepad 1 START), robot-centric override (Gamepad 1 right bumper), and D-pad translation.
- [ ] Test intake forward/reverse, ramp movement, release reverse pulse, and feed passage sensors.
- [ ] Spin up launcher (Gamepad 2 X), verify speed and green ready indicator, then stop (Gamepad 2 Y).
- [ ] Test a full feed/kicker cycle and confirm it returns to IDLE without jams. Feeding is **not** blocked by low flywheel speed.
- [ ] Verify STOP / OpMode exit actually turns **all** motors off, including flywheel. Shared `Robot.shutdown()` does not explicitly stop the flywheel; autonomous routines currently stop it in their own finally blocks.
- [ ] Validate both turret alliances on the field. BLUE TX offsets and all-area TY speed calibration are provisional.
- [ ] Drive each autonomous route on the actual field with a clear area. Confirm start poses, heading continuity, scoring accuracy, final park, and STOP behavior.
- [ ] Specifically verify Blue Right's documented heading mismatch at the first collection trajectory and 2-inch park starting-pose difference.
- [ ] Test fallback: lost AprilTags, bad camera frames, blocked distance sensor, and ball jams.
- [ ] Freeze tested Dashboard PIDF/vision values into source if necessary; Dashboard edits may not persist after restart.
- [ ] Keep a known-working build and an agreed fallback OpMode for match day.

## Driver quick reference
- **Gamepad 1:** left stick translation, right stick X rotation, right bumper robot-centric, D-pad controlled translation, START heading reset, A intake, B reverse, left trigger/bumper ramp down (release to raise).
- **Gamepad 2:** X flywheel on, Y flywheel off, right bumper automatic feed, A intake, B reverse.
- **Feedback:** green flashing = launcher within 30 ticks/s of target; red = not ready; yellow flashes = four intake sensor passages counted; one Gamepad 2 rumble when launcher first becomes ready.

## Outstanding software review items (do not change untested match behavior casually)
1. Add explicit flywheel stop on TeleOp shutdown and verify on hardware.
2. Validate/limit turret servo travel based on measured physical clearance, not just 0–1 command range.
3. Validate all autonomous trajectory continuity and ensure STOP interrupts long-running action sequences as expected.
4. Confirm launcher feed behavior if no ball is detected, and verify operator waits for flywheel readiness.
5. Recalibrate BLUE turret offsets and vision speed lookup where needed.

## Scope
This checklist and accompanying comments do not change control mappings, motor commands, trajectories, PIDF constants, servo settings, or camera tuning. All items require hands-on verification by the team.
