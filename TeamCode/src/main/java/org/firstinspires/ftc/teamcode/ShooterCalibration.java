package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import global.first.IgnitingInnovationGameDatabase;

import java.util.ArrayDeque;
import java.util.Deque;
import java.util.List;

/**
 * Calibration OpMode: the driver moves the robot manually to aim.
 * Controls the shooter by VELOCITY (encoder), not raw power, so the shot
 * doesn't vary as the battery voltage drops during the match.
 * MANUAL mode: DPAD up/down adjusts the target velocity by hand, to log
 * real (range, velocity) pairs shot by shot.
 * AUTO mode: interpolates those pairs over the CALIBRATION table.
 */
@TeleOp(name = "Shooter Calibration", group = "Pruebas")
public class ShooterCalibration extends LinearOpMode {

    // REV-41-1291 HD Hex Motor (no gearbox): 28 ticks/rev, 6000 RPM free at 12V.
    private static final double MOTOR_TICKS_PER_REV = 28.0;
    // Kept well below the 6000 RPM free speed to leave headroom for the
    // velocity controller and avoid maxing out the motor.
    private static final double MAX_SHOOTER_RPM = 5000.0;

    private static final int[] TARGET_TAG_IDS = {100, 101, 102, 103, 104};

    // Calibration points (range in cm -> target motor RPM).
    // Start with 2 and add more as you test intermediate distances.
    private static final double[][] CALIBRATION = {
            {100.0, 2200.0},
            {300.0, 3400.0},
    };

    private static final double MANUAL_STEP_RPM = 25.0; // change per press in manual mode
    private static final double VELOCITY_READY_TOLERANCE = 0.05; // 5% tolerance to consider "ready"
    private static final int RANGE_SAMPLES = 5; // frames averaged to smooth out the detected range

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private DcMotorEx shooter;

    private final Deque<Double> rangeHistory = new ArrayDeque<>();

    @Override
    public void runOpMode() {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setDirection(DcMotorEx.Direction.FORWARD);

        aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(IgnitingInnovationGameDatabase.getIgnitingInnovationTagLibrary())
                .build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        telemetry.addLine("Listo. Esperando start...");
        telemetry.addLine("Y = alternar AUTO/MANUAL, DPAD arriba/abajo = ajustar RPM manual");
        telemetry.addLine("Bumper derecho = disparar (mantener), A = detener shooter");
        telemetry.update();

        waitForStart();

        boolean autoMode = true;
        double manualRpm = 2500.0;
        boolean lastY = false;

        while (opModeIsActive()) {
            if (gamepad1.y && !lastY) autoMode = !autoMode;
            lastY = gamepad1.y;

            if (gamepad1.dpad_up) manualRpm = Math.min(MAX_SHOOTER_RPM, manualRpm + MANUAL_STEP_RPM);
            if (gamepad1.dpad_down) manualRpm = Math.max(0.0, manualRpm - MANUAL_STEP_RPM);

            AprilTagDetection target = findClosestTarget();
            Double rawRange = (target != null && target.ftcPose != null) ? target.ftcPose.range : null;
            Double smoothedRange = updateRangeHistory(rawRange);

            double targetRpm;
            if (autoMode && smoothedRange != null) {
                targetRpm = shooterRpmForRange(smoothedRange);
            } else {
                targetRpm = manualRpm;
            }
            targetRpm = Math.min(MAX_SHOOTER_RPM, targetRpm);

            boolean running = gamepad1.right_bumper && !gamepad1.a;
            double targetTicksPerSec = rpmToTicksPerSec(running ? targetRpm : 0.0);
            shooter.setVelocity(targetTicksPerSec);

            double currentRpm = ticksPerSecToRpm(shooter.getVelocity());
            boolean atSpeed = running && targetRpm > 0
                    && Math.abs(currentRpm - targetRpm) <= targetRpm * VELOCITY_READY_TOLERANCE;

            telemetry.addData("Modo", autoMode ? "AUTO" : "MANUAL");
            telemetry.addData("Tag", target != null ? target.id : "ninguno");
            telemetry.addData("Range (cm)", smoothedRange != null ? String.format("%.1f", smoothedRange) : "n/a");
            telemetry.addData("RPM objetivo", "%.0f", targetRpm);
            telemetry.addData("RPM actual", "%.0f", currentRpm);
            telemetry.addData("Estado", running ? (atSpeed ? "LISTO" : "acelerando...") : "detenido");
            telemetry.update();
        }

        shooter.setVelocity(0);
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
    // If there's no detection in the current frame, clear the history (no point averaging with stale data).
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

    // Piecewise linear interpolation over the CALIBRATION table
    private double shooterRpmForRange(double rangeCm) {
        if (rangeCm <= CALIBRATION[0][0]) return CALIBRATION[0][1];
        if (rangeCm >= CALIBRATION[CALIBRATION.length - 1][0]) return CALIBRATION[CALIBRATION.length - 1][1];

        for (int i = 0; i < CALIBRATION.length - 1; i++) {
            double r0 = CALIBRATION[i][0], p0 = CALIBRATION[i][1];
            double r1 = CALIBRATION[i + 1][0], p1 = CALIBRATION[i + 1][1];
            if (rangeCm >= r0 && rangeCm <= r1) {
                double t = (rangeCm - r0) / (r1 - r0);
                return p0 + t * (p1 - p0);
            }
        }
        return CALIBRATION[0][1];
    }

    private double rpmToTicksPerSec(double rpm) {
        return rpm * MOTOR_TICKS_PER_REV / 60.0;
    }

    private double ticksPerSecToRpm(double ticksPerSec) {
        return ticksPerSec * 60.0 / MOTOR_TICKS_PER_REV;
    }
}
