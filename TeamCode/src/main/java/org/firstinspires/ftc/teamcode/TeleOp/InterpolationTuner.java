package org.firstinspires.ftc.teamcode.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name = "🎯 INTERPOLATION TUNER (Camera)", group = "Tuning")
public class InterpolationTuner extends OpMode {

    private SimpleTurret turret;
    private Shooter shooter;
    private Intake intake;
    private Vision vision;
    private Hood hood;
    private Claw claw;
    private Follower follower; // Нужно только для инициализации шасси

    // === ПЕРЕМЕННЫЕ ДЛЯ НАСТРОЙКИ ===
    private double currentHoodPos = 0.35; // Старт
    private double currentRPM = 3500;     // Старт

    // Тег, на который смотрим (Красная корзина)
    private static final int TARGET_TAG = 24;

    // Анти-дребезг кнопок
    private boolean lastUp = false, lastDown = false;
    private boolean lastLeft = false, lastRight = false;

    private boolean cameraSetup = false;

    @Override
    public void init() {
        // Инициализируем, чтобы робот не падал с ошибкой
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0,0,0));

        turret = new SimpleTurret(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        vision = new Vision(hardwareMap);
        hood = new Hood(hardwareMap);
        claw = new Claw(hardwareMap);

        claw.close();

        // ВАЖНО: Блокируем башню на 0 градусов
        turret.setTargetAngle(0);

        telemetry.addLine("1. Наведи робота на тег " + TARGET_TAG);
        telemetry.addLine("2. D-PAD UP/DOWN: Худ +/- 0.005");
        telemetry.addLine("3. D-PAD L/R: RPM +/- 50");
        telemetry.addLine("4. Курком стреляй");
    }

    @Override
    public void init_loop() {
        if (!cameraSetup) cameraSetup = vision.applyCombatSettings();
        if (cameraSetup) telemetry.addData("Vision", "✅ READY (Exposure Low)");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        // Обновляем системы
        follower.update(); // Для езды
        shooter.update();  // Для PID

        // Башня всегда держит 0
        turret.update(follower.getPose(), null);

        // === 1. ВИЖН (Получаем дистанцию) ===
        AprilTagDetection tag = vision.getTarget(TARGET_TAG);
        double cameraDistance = -1;

        if (tag != null) {
            cameraDistance = tag.ftcPose.range; // Дистанция в дюймах
        }

        // === 2. УПРАВЛЕНИЕ ХУДОМ (Точная настройка) ===
        // Шаг 0.005 для супер-точности
        if (gamepad1.dpad_up && !lastUp) currentHoodPos += 0.005;
        if (gamepad1.dpad_down && !lastDown) currentHoodPos -= 0.005;

        currentHoodPos = Range.clip(currentHoodPos, 0.2, 0.65); // Лимиты худа
        lastUp = gamepad1.dpad_up;
        lastDown = gamepad1.dpad_down;

        // === 3. УПРАВЛЕНИЕ RPM ===
        if (gamepad1.dpad_right && !lastRight) currentRPM += 50;
        if (gamepad1.dpad_left && !lastLeft) currentRPM -= 50;

        currentRPM = Range.clip(currentRPM, 0, 4500);
        lastRight = gamepad1.dpad_right;
        lastLeft = gamepad1.dpad_left;

        // === 4. ПРИМЕНЕНИЕ (ACTUATION) ===
        shooter.setTargetRPM(currentRPM);

        hood.setBasePosition(currentHoodPos);
        hood.update(shooter.getCurrentRPM(), currentRPM);

        // === 5. СТРЕЛЬБА ===
        if (gamepad1.right_trigger > 0.1) {
            claw.open();
            intake.intake();
        } else {
            claw.close();
            intake.stop();
        }

        // Езда (чтобы отъезжать)
        follower.setTeleOpDrive(-gamepad1.left_stick_y * 0.5, -gamepad1.left_stick_x * 0.5, (gamepad1.left_trigger - gamepad1.right_trigger) * 0.3, false);

        // === ТЕЛЕМЕТРИЯ (ДЛЯ ЗАПИСИ В ТАБЛИЦУ) ===
        telemetry.addLine("=== КАЛИБРОВКА ===");
        if (tag != null) {
            // Эту цифру пиши в shotCalculator.addCalibration(DIST, ...)
            telemetry.addData("📏 DISTANCE (Inch)", "%.2f", cameraDistance);
        } else {
            telemetry.addData("📏 DISTANCE", "🔴 NO TAG");
        }

        // Эту цифру пиши вторым аргументом: addCalibration(..., HOOD)
        telemetry.addData("📐 HOOD POS", "%.3f", currentHoodPos);

        telemetry.addData("⚙️ RPM", "%.0f", currentRPM);
        telemetry.update();
    }
}