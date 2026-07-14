# FGC2025 Team Code — Computer Vision & Shooter Control

Robot code for FIRST Global Challenge, built on the official FTC SDK (DECODE 2025-2026 season base). This fork adds AprilTag-based computer vision to aim and calibrate the shooter, plus auto-alignment driving assists.

## Hardware

- **Controller**: REV Robotics Control Hub
- **Camera**: Logitech C270 (USB, plugged into the Control Hub's USB 3.0 port), mounted on the **back** of the robot alongside the shooter
- **Shooter motor**: REV-41-1291 HD Hex Motor (no gearbox) — 28 ticks/rev, 6000 RPM free speed at 12V, driven in closed-loop velocity mode (`RUN_USING_ENCODER`), plugged into port 3 of the Expansion Hub, hardware-mapped as `motorUltra`
- **AprilTags**: FIRST Global 2026 field tags (IDs 100–104), detected via the official `IgnitingInnovationAprilTagLibrary-1.0.jar` (see `TeamCode/libs/`), which supplies the correct real-world tag size for accurate pose/range estimation

## OpModes

### `MovementShooterAuto` (competition TeleOp)
Full driver-controlled OpMode: omnidirectional chassis, collector, encoder-controlled rope/lift, extender servos, and the AprilTag-aimed shooter, plus three auto-alignment assists.

**Gamepad 1 — chassis & auto-aim:**
| Control | Action |
|---|---|
| Left stick | Forward/back + strafe |
| Right stick | Rotate |
| Right trigger | Slow mode (half power) |
| **DPAD right** (hold) | Auto-center: rotates in place until the AprilTag bearing is 0° |
| **Triangle** (hold) | Auto-approach: drives to hold a fixed distance (30cm) from the AprilTag |
| **Square** (hold) | Auto-lateral: strafes until laterally aligned with the AprilTag |
| Left bumper | Toggle shooter AUTO/MANUAL mode |
| DPAD up/down | Adjust manual shooter RPM (MANUAL mode only) |
| Right bumper (hold) | Spin up shooter to target velocity |
| Circle (hold, no right bumper) | Reverse shooter at low power to clear a jam |

**Gamepad 2 — mechanisms:** collector (cross/triangle toggle), rope lift with encoder hold (D-pad, circle to lock/unlock, square = timed 3s retract-and-lock), extender servos (bumpers/triggers).

Because the camera/shooter face the **back** of the robot, all three auto-aim assists are signed so that driving "forward" on the stick still matches the driver's intuition, while the underlying math corrects for the tag effectively being behind the chassis.

The shooter is controlled by **velocity (RPM via encoder)**, not raw motor power, so shots stay consistent as the battery drains over a match — a fixed `power` value would push the game piece less far as voltage sags, but a target RPM is actively held by the motor's closed-loop control regardless of battery level (until the motor physically can't reach it).

### `ShooterCalibration` (test/tuning TeleOp)
Standalone OpMode used to build the distance→RPM calibration table. Displays live AprilTag range/bearing telemetry and lets you dial in a manual RPM with the D-pad to find the value that reliably scores at a given distance, without driving the rest of the robot.

## Shooter Calibration

Both OpModes interpolate shooter RPM from a `{range_cm, rpm}` table (`SHOOTER_CALIBRATION` / `CALIBRATION`). To (re)calibrate:

1. Run `ShooterCalibration`, position the robot at a known distance from an AprilTag.
2. In MANUAL mode, use D-pad up/down to find the RPM that reliably scores at that distance (test several shots).
3. Repeat at 4–5 distances spanning your realistic shooting range.
4. Update the table with the real `{distance, rpm}` pairs, sorted ascending by distance.
5. Keep values below `MAX_SHOOTER_RPM` (5000) — well under the motor's 6000 RPM free-speed ceiling, to leave headroom for the velocity controller.

## Auto-Aim Tuning

Each auto-alignment feature (center / approach / lateral) uses a simple proportional controller with a **minimum power floor** (to overcome chassis friction on small errors) and a **deadband** (to stop correcting once "close enough", avoiding oscillation) plus a slew-rate ramp (to avoid jerky power steps). If an assist oscillates or doesn't move, tune its `_KP` (gain), `MIN_..._POWER` (floor), and `..._DEADBAND` constants at the top of `MovementShooterAuto.java`.

## Project Setup (VS Code + Gradle, no Android Studio)

This project builds with the standard Gradle wrapper — Android Studio is not required.

1. Install a JDK (17+), the Android SDK command-line tools (`platform-tools`, a matching `platforms;android-XX` and `build-tools;XX.X.X`), and point `local.properties` (`sdk.dir=...`) at the SDK root.
2. Build: `./gradlew.bat assembleDebug`
3. Deploy: connect to the Control Hub via USB or WiFi (`adb connect <ip>:5555`), then:
   ```
   adb -s <device> install -r "TeamCode\build\outputs\apk\debug\TeamCode-debug.apk"
   ```

See the original FTC SDK docs (linked from the FIRST Tech Challenge / FIRST Global program) for Driver Station pairing, robot configuration, and general OpMode programming basics — this README only covers what's specific to this team's code.
