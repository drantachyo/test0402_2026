package org.firstinspires.ftc.teamcode.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.math.ShotCalculator;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name = "🚫 COMBAT: NO TURRET", group = "Combat")
public class    CombatTeleOpNoTurret extends OpMode {

    private Follower follower;
    private Shooter shooter;
    private Intake intake;
    private Vision vision;
    private Hood hood;
    private Claw claw;

    private ShotCalculator shotCalculator;

    public enum RobotState { IDLE, INTAKE, OUTTAKE, PREP_SHOOT, SHOOTING }
    private RobotState currentState = RobotState.IDLE;

    private static final double RED_BASKET_X = 130, RED_BASKET_Y = 130;
    private static final double BLUE_BASKET_X = 130, BLUE_BASKET_Y = 14;
    private static final Pose RESET_POSE = new Pose(8, 8, 0);

    private boolean isRedAlliance = true;
    private int targetTagId = 24;
    private double targetX = RED_BASKET_X, targetY = RED_BASKET_Y;

    // Флаг настройки камеры
    private boolean cameraSetup = false;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));

        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        vision = new Vision(hardwareMap);
        hood = new Hood(hardwareMap);
        claw = new Claw(hardwareMap);
        shotCalculator = new ShotCalculator();

        claw.close();
        telemetry.addLine("NO TURRET MODE. Waiting for camera...");
    }

    @Override
    public void init_loop() {
        // 1. Пытаемся настроить камеру (2ms/240)
        if (!cameraSetup) {
            cameraSetup = vision.applyCombatSettings();
        }

        // 2. Выбор альянса
        if (gamepad1.b) {
            isRedAlliance = true;
            targetTagId = 24; // Red Basket ID
            targetX = RED_BASKET_X; targetY = RED_BASKET_Y;
        }
        if (gamepad1.x) {
            isRedAlliance = false;
            targetTagId = 20; // Blue Basket ID
            targetX = BLUE_BASKET_X; targetY = BLUE_BASKET_Y;
        }

        if (cameraSetup) telemetry.addData("Vision", "✅ READY (2ms/240)");
        else telemetry.addData("Vision", "⏳ Setting Exposure...");

        telemetry.addData("Alliance", isRedAlliance ? "RED" : "BLUE");
        telemetry.update();
    }

    @Override
    public void start() { follower.startTeleopDrive(); }

    @Override
    public void loop() {
        follower.update();
        // ВАЖНО: Обновляем шутер, чтобы применять новые PIDF из Panels
        shooter.update();

        Pose pose = follower.getPose();
        AprilTagDetection tag = vision.getTarget(targetTagId);

        if (gamepad1.options) {
            follower.setPose(RESET_POSE);
        }

        boolean aim = gamepad2.left_trigger > 0.1;
        boolean fire = gamepad2.right_trigger > 0.1;

        switch (currentState) {
            case IDLE:
                stopAll();
                if (aim) currentState = RobotState.PREP_SHOOT;
                else if (gamepad2.b) currentState = RobotState.INTAKE;
                else if (gamepad2.dpad_down) currentState = RobotState.OUTTAKE;
                break;

            case INTAKE:
                intake.intake(); claw.close();
                if (!gamepad2.b) currentState = RobotState.IDLE;
                break;

            case OUTTAKE:
                intake.outtake(); claw.close();
                if (!gamepad2.dpad_down) currentState = RobotState.IDLE;
                break;

            case PREP_SHOOT:
                shooter.setTargetRPM(3500);
                hood.update(shooter.getCurrentRPM(), 0); // Худ лежит

                if (!aim) currentState = RobotState.IDLE;
                    // Стреляем ТОЛЬКО если шутер разогнался (isReady)
                else if (fire && shooter.isReady()) currentState = RobotState.SHOOTING;
                break;

            case SHOOTING:
                shooter.setTargetRPM(3500);

                // Авто-Худ
                hood.setBasePosition(0.35);
                hood.update(shooter.getCurrentRPM(), 3500);

                claw.open(); intake.intake();

                if (!aim || !fire) currentState = aim ? RobotState.PREP_SHOOT : RobotState.IDLE;
                break;
        }

        handleChassis();

        telemetry.addData("State", currentState);
        telemetry.addData("Shooter RPM", "%.0f / 3500", shooter.getCurrentRPM());
        telemetry.addData("Ready?", shooter.isReady());
        telemetry.update();
    }

    private void stopAll() {
        shooter.setTargetRPM(0);
        intake.stop();
        claw.close();
        hood.update(0, 0);
    }

    private void handleChassis() {
        double drive = Math.pow(-gamepad1.left_stick_y, 3);
        double strafe = Math.pow(-gamepad1.left_stick_x, 3);
        double turn = Math.pow(gamepad1.left_trigger - gamepad1.right_trigger, 3);

        if (gamepad1.right_bumper) { drive *= 0.3; strafe *= 0.3; turn *= 0.3; }
        follower.setTeleOpDrive(drive, strafe, turn, false);
    }
}