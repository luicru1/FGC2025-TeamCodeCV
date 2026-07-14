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
 * OpMode de calibracion: el driver mueve el robot manualmente para apuntar.
 * Controla el shooter por VELOCIDAD (encoder), no por potencia cruda, para que
 * el tiro no varie cuando baja el voltaje de la bateria durante el partido.
 * Modo MANUAL: DPAD arriba/abajo ajusta la velocidad objetivo a mano, para ir
 * anotando pares reales (range, velocidad) tiro por tiro.
 * Modo AUTO: interpola esos pares sobre la tabla CALIBRATION.
 */
@TeleOp(name = "Shooter Calibration", group = "Pruebas")
public class ShooterCalibration extends LinearOpMode {

   
    // (viene en la hoja de especificaciones del motor, ej. goBILDA/REV Robotics).
    private static final double MOTOR_TICKS_PER_REV = 28.0;

    private static final int[] TARGET_TAG_IDS = {100, 101, 102, 103, 104};

    // Puntos de calibracion (range en cm -> velocidad objetivo en RPM del motor).
    // Empieza con 2 y ve agregando mas conforme pruebes distancias intermedias.
    private static final double[][] CALIBRATION = {
            {100.0, 2200.0},
            {300.0, 3400.0},
    };

    private static final double MANUAL_STEP_RPM = 25.0; // cambio por pulsacion en modo manual
    private static final double VELOCITY_READY_TOLERANCE = 0.05; // 5% de margen para considerar "listo"
    private static final int RANGE_SAMPLES = 5; // frames a promediar para suavizar el range detectado

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

            if (gamepad1.dpad_up) manualRpm += MANUAL_STEP_RPM;
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

    // Promedio movil de las ultimas RANGE_SAMPLES lecturas, para suavizar el jitter del AprilTag.
    // Si no hay deteccion en el frame actual, se limpia el historial (no interesa promediar con datos viejos).
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

    // Interpolacion lineal por tramos sobre la tabla CALIBRATION
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
