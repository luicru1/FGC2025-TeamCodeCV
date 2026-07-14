package org.firstinspires.ftc.teamcode;

//--------------------------------------------------------------
// Authors: Team Guatemala & Luis Cruz (Computer Vision Related)
//--------------------------------------------------------------

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import global.first.IgnitingInnovationGameDatabase;

import java.util.ArrayDeque;
import java.util.Deque;
import java.util.List;

@TeleOp(name = "Movement 2 - Shooter Auto", group = "Competition")
public class MovementShooterAuto extends LinearOpMode {

    // Hardware Declaration

    // Traction motors (omnidirectional)
    private DcMotorEx rightFront, rightBack, leftFront, leftBack;

    // Mechanism actuators
    private DcMotorEx motorHex;  // Collector motor
    private DcMotorEx motorHex2; // Rope motor (uses encoder)
    private DcMotorEx motorUltra; // Shooter motor (Expansion Hub port 3)

    // Servomotors
    private Servo servomotor2; // Right extender
    private Servo servomotor3; // Left extender

    // Vision (shooter aiming reference)
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    // State Variables

    // Logical states (for toggles/on-off)
    private boolean hexForward = false;
    private boolean hexReverse = false;
    private boolean ropeHoldActive = false; // Rope lock

    // Debounce: prevents repeatedly activating functions with a single press
    private boolean prevX, prevTriangle, prevCircle2, prevLeftBumper;

    // Constants

    private static final double HEX_POWER = 1.0;   // Collector power
    private static final double HOLD_POWER = 0.45; // Force to maintain rope position

    // Positions for servos
    private static final double SERVO_ADELANTE = 1.0;//Forward
    private static final double SERVO_ATRAS = 0.0;//Backward
    private static final double SERVO_PARAR = 0.5;//Stop

    // Stores the current rope position when locked
    private int holdPosition = 0;

    // Shooter (motorUltra) - velocity-based, aimed via AprilTag distance

    // REV-41-1291 HD Hex Motor (no gearbox): 28 ticks/rev at the output shaft.
    private static final double MOTOR_TICKS_PER_REV = 28.0;

    private static final int[] TARGET_TAG_IDS = {100, 101, 102, 103, 104};

    // Calibration points (range in cm -> target motor RPM). Fill in with the
    // real pairs measured in Shooter Calibration, sorted by ascending range.
    private static final double[][] SHOOTER_CALIBRATION = {
            {100.0, 2200.0},
            {300.0, 3400.0},
    };

    private static final double MANUAL_STEP_RPM = 25.0;
    // REV-41-1291 spins freely at 6000 RPM unloaded; kept well below that
    // to leave headroom for the velocity controller and avoid maxing out the motor.
    private static final double MAX_SHOOTER_RPM = 5000.0;
    private static final double VELOCITY_READY_TOLERANCE = 0.05; // 5%
    private static final int RANGE_SAMPLES = 5; // frames averaged to smooth out the range reading

    private static final double DECLOG_POWER = -0.3; // fixed power used to clear a shooter jam

    // Chassis auto-center, triggered by DPAD right (gamepad1)
    private static final double AUTO_CENTER_KP = 0.02;       // proportional gain (degrees of error -> turn power)
    private static final double AUTO_CENTER_MIN_POWER = 0.13; // power floor to overcome friction at close range (small error)
    private static final double AUTO_CENTER_MAX_POWER = 0.4; // turn power cap, avoids violent spins
    private static final double AUTO_CENTER_DEADBAND_DEG = 2.5; // tolerance to consider "already centered"
    private static final double AUTO_CENTER_MAX_ACCEL = 0.05; // max power change per cycle, keeps it smooth

    // Chassis auto-approach, triggered by "triangle" (gamepad1): holds the
    // robot at TARGET_RANGE_CM from the AprilTag.
    private static final double TARGET_RANGE_CM = 80.0;
    private static final double RANGE_KP = 0.035;         // proportional gain (cm of error -> drive power)
    private static final double MIN_RANGE_POWER = 0.18;   // power floor to overcome chassis friction/weight
    private static final double MAX_RANGE_POWER = 0.65;   // drive power cap, avoids hard bumps
    private static final double RANGE_DEADBAND_CM = 3.0;  // tolerance to consider "already at distance"
    private static final double RANGE_MAX_ACCEL = 0.05;   // max power change per cycle, keeps it smooth

    // Chassis auto-lateral, triggered by "square" (gamepad1): strafes until
    // laterally aligned with the AprilTag (ftcPose.x = 0).
    private static final double LATERAL_KP = 0.025;
    private static final double MIN_LATERAL_POWER = 0.3;  // power floor: strafing costs more than driving/turning (roller friction)
    private static final double MAX_LATERAL_POWER = 0.7;
    private static final double LATERAL_DEADBAND_CM = 5.0; // wider than range/bearing: the high floor overshoots a tight band
    private static final double LATERAL_MAX_ACCEL = 0.05; // max power change per cycle, keeps it smooth

    private final Deque<Double> rangeHistory = new ArrayDeque<>();
    private boolean shooterAutoMode = true;
    private double shooterManualRpm = 2500.0;
    private double rangePowerSmoothed = 0.0;  // last applied auto-approach power, for smoothing
    private double centerPowerSmoothed = 0.0; // last applied auto-center power, for smoothing
    private double lateralPowerSmoothed = 0.0; // last applied auto-lateral power, for smoothing


    // Main Method
    @Override
    public void runOpMode() throws InterruptedException {

        // Initialization

        // Map hardware names (from the configuration in the Driver Hub)
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");

        motorHex = hardwareMap.get(DcMotorEx.class, "motorHex");
        motorHex2 = hardwareMap.get(DcMotorEx.class, "motorHex2");
        motorUltra = hardwareMap.get(DcMotorEx.class, "motorUltra"); // must be in Expansion Hub port 3

        servomotor2 = hardwareMap.get(Servo.class, "servomotor2");
        servomotor3 = hardwareMap.get(Servo.class, "servomotor3");

        // Left side motors must be inverted so they all turn in the same direction
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        // Configure the rope (motor with encoder)
        motorHex2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // brakes when released
        motorHex2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorHex2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Configure the shooter for closed-loop velocity control
        motorUltra.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorUltra.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorUltra.setDirection(DcMotorEx.Direction.FORWARD);

        // Servos in neutral position at start
        servomotor2.setPosition(SERVO_PARAR);
        servomotor3.setPosition(SERVO_PARAR);

        // Vision portal for AprilTag-based shooter distance
        aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(IgnitingInnovationGameDatabase.getIgnitingInnovationTagLibrary())
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        // Message on the Driver Hub screen
        telemetry.addLine("[System Ready] Press PLAY to start.");
        telemetry.update();

        // Waits until the user presses PLAY
        waitForStart();

        // Main Loop
        while (opModeIsActive()) {

            // Omnidirectional Movement (gamepad 1 joysticks)

            float powerY = -gamepad1.left_stick_y; // Forward / backward
            float powerX = gamepad1.left_stick_x; // Lateral movement
            float rot = gamepad1.right_stick_x;  // Rotation

            // AprilTag detection (used for chassis auto-centering and for the shooter)
            AprilTagDetection target = findClosestTarget();
            Double rawRange = (target != null && target.ftcPose != null) ? target.ftcPose.range : null;
            Double smoothedRange = updateRangeHistory(rawRange);

            // Auto-center: DPAD right (gamepad1) rotates the chassis only, until
            // the AprilTag bearing is aligned to 0, without touching forward/lateral
            // movement, which the driver still controls with the left stick.
            boolean autoCenterActive = gamepad1.dpad_right;
            double centerTargetPower;
            if (autoCenterActive) {
                if (target != null && target.ftcPose != null) {
                    double bearingError = target.ftcPose.bearing; // degrees, + = tag to the right
                    if (Math.abs(bearingError) > AUTO_CENTER_DEADBAND_DEG) {
                        double magnitude = Range.clip(Math.abs(bearingError) * AUTO_CENTER_KP, AUTO_CENTER_MIN_POWER, AUTO_CENTER_MAX_POWER);
                        centerTargetPower = -Math.copySign(magnitude, bearingError);
                    } else {
                        centerTargetPower = 0; // already centered, stop turning
                    }
                } else {
                    centerTargetPower = 0; // no tag visible, don't turn blindly
                }
            } else {
                centerTargetPower = 0;
            }
            centerPowerSmoothed = rampTowards(centerPowerSmoothed, centerTargetPower, AUTO_CENTER_MAX_ACCEL);
            if (autoCenterActive) rot = (float) centerPowerSmoothed;

            // Auto-approach: "triangle" (gamepad1) moves the chassis in Y only,
            // until it's at TARGET_RANGE_CM from the AprilTag. Lateral and rotation
            // (or the auto-center above) remain available at the same time.
            // Bound to a face button instead of dpad_left because a physical
            // D-pad can't register left and right pressed at the same time.
            // Camera/shooter are mounted on the BACK of the robot: getting closer
            // to the tag means driving the chassis in reverse, hence the inverted sign.
            boolean autoRangeActive = gamepad1.triangle;
            double rangeTargetPower;
            if (autoRangeActive) {
                if (target != null && target.ftcPose != null && smoothedRange != null) {
                    double rangeError = smoothedRange - TARGET_RANGE_CM; // + = too far, - = too close
                    if (Math.abs(rangeError) > RANGE_DEADBAND_CM) {
                        double magnitude = Range.clip(Math.abs(rangeError) * RANGE_KP, MIN_RANGE_POWER, MAX_RANGE_POWER);
                        rangeTargetPower = -Math.copySign(magnitude, rangeError);
                    } else {
                        rangeTargetPower = 0; // already at the target distance
                    }
                } else {
                    rangeTargetPower = 0; // no tag visible, don't drive blindly
                }
            } else {
                rangeTargetPower = 0;
            }
            // Smooth the power change (ramp) instead of jumping straight to the target value
            rangePowerSmoothed = rampTowards(rangePowerSmoothed, rangeTargetPower, RANGE_MAX_ACCEL);
            if (autoRangeActive) powerY = (float) rangePowerSmoothed;

            // Auto-lateral: "square" (gamepad1) strafes the chassis until it's
            // laterally aligned with the AprilTag (x = 0). Just like auto-approach,
            // the sign is inverted because the camera is mounted on the back
            // (left/right appear mirrored from the camera's point of view).
            boolean autoLateralActive = gamepad1.square;
            double lateralTargetPower;
            if (autoLateralActive) {
                if (target != null && target.ftcPose != null) {
                    double lateralError = target.ftcPose.x; // cm, + = tag to the camera's right
                    if (Math.abs(lateralError) > LATERAL_DEADBAND_CM) {
                        double magnitude = Range.clip(Math.abs(lateralError) * LATERAL_KP, MIN_LATERAL_POWER, MAX_LATERAL_POWER);
                        lateralTargetPower = -Math.copySign(magnitude, lateralError);
                    } else {
                        lateralTargetPower = 0; // already laterally aligned
                    }
                } else {
                    lateralTargetPower = 0; // no tag visible, don't strafe blindly
                }
            } else {
                lateralTargetPower = 0;
            }
            lateralPowerSmoothed = rampTowards(lateralPowerSmoothed, lateralTargetPower, LATERAL_MAX_ACCEL);
            if (autoLateralActive) powerX = (float) lateralPowerSmoothed;

            // Combinations for mecanum/omni wheels
            float RF = powerY - powerX - rot; // Right front wheel
            float RB = powerY + powerX - rot; // Right rear wheel
            float LF = powerY + powerX + rot; // Left front wheel
            float LB = powerY - powerX + rot; // Left rear wheel

           // Collector (gamepad2: x and triangle)
            boolean x = gamepad2.cross;
            boolean triangle = gamepad2.triangle;

            // Toggle direction with one touch (on-off)
            if (x && !prevX) {
                hexForward = !hexForward;
                hexReverse = false;
            } else if (triangle && !prevTriangle) {
                hexReverse = !hexReverse;
                hexForward = false;
            }

            // Execute according to state
            if (hexForward) {
                motorHex.setPower(HEX_POWER);// Collector system spinning clockwise
            } else if (hexReverse) {
                motorHex.setPower(-HEX_POWER);// Collector system spinning counter-clockwise
            } else {
                motorHex.setPower(0);// Remain in neutral state
            }

            // Rope with encoder (gamepad 2; D-pad and circle)
            boolean dpadLeft = gamepad2.dpad_left;
            boolean dpadRight = gamepad2.dpad_right;
            boolean circle2 = gamepad2.circle;

            // Hold mode (position lock)
            if (circle2 && !prevCircle2) {
                ropeHoldActive = !ropeHoldActive;

                if (ropeHoldActive) {
                    // Save current position and hold it
                    holdPosition = motorHex2.getCurrentPosition();
                    motorHex2.setTargetPosition(holdPosition);
                    motorHex2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motorHex2.setPower(HOLD_POWER);
                } else {
                    // Release manual control
                    motorHex2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    motorHex2.setPower(0);
                }
            }

            // Manual rope control (only if not in hold mode)
            if (!ropeHoldActive) {
                if (dpadLeft) { // Up
                    motorHex2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    motorHex2.setPower(1.0);
                } else if (dpadRight) { // Down
                    motorHex2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    motorHex2.setPower(-1.0);
                } else {
                    motorHex2.setPower(0);
                }
            } else {
                // Maintain locked position
                motorHex2.setTargetPosition(holdPosition);
                motorHex2.setPower(HOLD_POWER);
            }

            boolean cuadrado = gamepad2.square;
            if (cuadrado) {
            motorHex2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorHex2.setPower(-1.0);


            sleep(3000);

            holdPosition = motorHex2.getCurrentPosition();
            motorHex2.setTargetPosition(holdPosition);
            motorHex2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorHex2.setPower(HOLD_POWER);

            telemetry.addLine("Rope moved for 3s and locked");
            telemetry.update();
}

            // Shooter (gamepad 1: left bumper toggles AUTO/MANUAL, dpad adjusts
            // manual RPM, right bumper holds to spin up and fire, circle declogs)

            boolean leftBumper = gamepad1.left_bumper;
            if (leftBumper && !prevLeftBumper) shooterAutoMode = !shooterAutoMode;

            if (gamepad1.dpad_up) shooterManualRpm = Math.min(MAX_SHOOTER_RPM, shooterManualRpm + MANUAL_STEP_RPM);
            if (gamepad1.dpad_down) shooterManualRpm = Math.max(0.0, shooterManualRpm - MANUAL_STEP_RPM);

            double shooterTargetRpm;
            if (shooterAutoMode && smoothedRange != null) {
                shooterTargetRpm = shooterRpmForRange(smoothedRange);
            } else {
                shooterTargetRpm = shooterManualRpm;
            }
            shooterTargetRpm = Math.min(MAX_SHOOTER_RPM, shooterTargetRpm);

            boolean shooterRunning = gamepad1.right_bumper && !gamepad1.circle;
            boolean shooterDeclog = gamepad1.circle && !gamepad1.right_bumper;

            if (shooterDeclog) {
                motorUltra.setPower(DECLOG_POWER);
            } else {
                double targetTicksPerSec = rpmToTicksPerSec(shooterRunning ? shooterTargetRpm : 0.0);
                motorUltra.setVelocity(targetTicksPerSec);
            }

            double shooterCurrentRpm = ticksPerSecToRpm(motorUltra.getVelocity());
            boolean shooterAtSpeed = shooterRunning && shooterTargetRpm > 0
                    && Math.abs(shooterCurrentRpm - shooterTargetRpm) <= shooterTargetRpm * VELOCITY_READY_TOLERANCE;

            // Extender Servos (gamepad 2: lb, rb, l2 and r2)
            boolean lb = gamepad2.left_bumper;
            boolean rb = gamepad2.right_bumper;
            float l2 = gamepad2.left_trigger;
            float r2 = gamepad2.right_trigger;

            // Possible extension and retraction combinations
            if (lb) {// Rotate left motor only - Clockwise
                servomotor3.setPosition(SERVO_ATRAS);
            } else if (rb) {// Rotate right motor only - Clockwise
                servomotor2.setPosition(SERVO_ADELANTE);
            } else if (l2 > 0) {// Rotate left motor only - Counter-clockwise
                servomotor3.setPosition(SERVO_ADELANTE);
            } else if (r2 > 0) {// Rotate right motor only - Counter-clockwise
                servomotor2.setPosition(SERVO_ATRAS);
            } else if (lb && rb) {// Rotate at the same time forward
                servomotor3.setPosition(SERVO_ATRAS);
                servomotor2.setPosition(SERVO_ADELANTE);
            } else if (l2 > 0 && r2 > 0) {// Rotate at the same time backward
                servomotor3.setPosition(SERVO_ADELANTE);
                servomotor2.setPosition(SERVO_ATRAS);
            } else {
                servomotor2.setPosition(SERVO_PARAR);
                servomotor3.setPosition(SERVO_PARAR);
            }

            float R2 = gamepad1.right_trigger;

            if (R2 > 0){
            rightFront.setPower(Range.clip(RF, -0.5, 0.5));
            rightBack.setPower(Range.clip(RB, -0.5, 0.5));
            leftFront.setPower(Range.clip(LF, -0.5, 0.5));
            leftBack.setPower(Range.clip(LB, -0.5, 0.5));
            }else {
            rightFront.setPower(Range.clip(RF, -1, 1));
            rightBack.setPower(Range.clip(RB, -1, 1));
            leftFront.setPower(Range.clip(LF, -1, 1));
            leftBack.setPower(Range.clip(LB, -1, 1));
            }

            // Telemetry, messages on the Driver Hub

            telemetry.addData("Hex Rotor", hexForward ? "→ Forward" : (hexReverse ? "← Reverse" : "Stopped"));
            telemetry.addData("Rope Pos (ticks)", motorHex2.getCurrentPosition());
            telemetry.addData("Rope Target", motorHex2.getTargetPosition());
            telemetry.addData("Hold", ropeHoldActive ? "Active" : "Free");
            telemetry.addData("Servo2", "%.2f", servomotor2.getPosition());
            telemetry.addData("Servo3", "%.2f", servomotor3.getPosition());

            telemetry.addData("Auto-Centro", autoCenterActive ? (target != null ? "centrando" : "sin tag") : "libre");
            telemetry.addData("Auto-Avance", autoRangeActive ? (target != null ? "ajustando" : "sin tag") : "libre");
            telemetry.addData("Auto-Lateral", autoLateralActive ? (target != null ? "ajustando" : "sin tag") : "libre");
            telemetry.addData("Shooter Modo", shooterAutoMode ? "AUTO" : "MANUAL");
            telemetry.addData("Shooter Tag", target != null ? target.id : "ninguno");
            telemetry.addData("Shooter Range (cm)", smoothedRange != null ? String.format("%.1f", smoothedRange) : "n/a");
            telemetry.addData("Shooter RPM objetivo", "%.0f", shooterTargetRpm);
            telemetry.addData("Shooter RPM actual", "%.0f", shooterCurrentRpm);
            telemetry.addData("Shooter Estado", shooterDeclog ? "destrabando" : (shooterRunning ? (shooterAtSpeed ? "LISTO" : "acelerando...") : "detenido"));
            telemetry.update();

            // Save previous states (debounce)
            prevX = x;
            prevTriangle = triangle;
            prevCircle2 = circle2;
            prevLeftBumper = leftBumper;
        }

        motorUltra.setVelocity(0);
        visionPortal.close();
    }

    private AprilTagDetection findClosestTarget() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        AprilTagDetection closest = null;
        for (AprilTagDetection detection : detections) {
            if (detection.ftcPose == null || !isTargetId(detection.id)) continue;
            if (closest == null || detection.ftcPose.range < closest.ftcPose.range) {
                closest = detection;
            }
        }
        return closest;
    }

    private boolean isTargetId(int id) {
        for (int targetId : TARGET_TAG_IDS) {
            if (targetId == id) return true;
        }
        return false;
    }

    // Moving average of the last RANGE_SAMPLES readings, to smooth out AprilTag jitter.
    private Double updateRangeHistory(Double rawRange) {
        if (rawRange == null) {
            rangeHistory.clear();
            return null;
        }
        rangeHistory.addLast(rawRange);
        if (rangeHistory.size() > RANGE_SAMPLES) rangeHistory.removeFirst();

        double sum = 0;
        for (double r : rangeHistory) sum += r;
        return sum / rangeHistory.size();
    }

    // Moves "current" toward "target" without exceeding maxDelta per call, to smooth power ramps
    private double rampTowards(double current, double target, double maxDelta) {
        double delta = target - current;
        if (Math.abs(delta) > maxDelta) delta = Math.copySign(maxDelta, delta);
        return current + delta;
    }

    // Piecewise linear interpolation over the SHOOTER_CALIBRATION table
    private double shooterRpmForRange(double rangeCm) {
        if (rangeCm <= SHOOTER_CALIBRATION[0][0]) return SHOOTER_CALIBRATION[0][1];
        if (rangeCm >= SHOOTER_CALIBRATION[SHOOTER_CALIBRATION.length - 1][0])
            return SHOOTER_CALIBRATION[SHOOTER_CALIBRATION.length - 1][1];

        for (int i = 0; i < SHOOTER_CALIBRATION.length - 1; i++) {
            double r0 = SHOOTER_CALIBRATION[i][0], p0 = SHOOTER_CALIBRATION[i][1];
            double r1 = SHOOTER_CALIBRATION[i + 1][0], p1 = SHOOTER_CALIBRATION[i + 1][1];
            if (rangeCm >= r0 && rangeCm <= r1) {
                double t = (rangeCm - r0) / (r1 - r0);
                return p0 + t * (p1 - p0);
            }
        }
        return SHOOTER_CALIBRATION[0][1];
    }

    private double rpmToTicksPerSec(double rpm) {
        return rpm * MOTOR_TICKS_PER_REV / 60.0;
    }

    private double ticksPerSecToRpm(double ticksPerSec) {
        return ticksPerSec * 60.0 / MOTOR_TICKS_PER_REV;
    }
}