package org.firstinspires.ftc.teamcode.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.math.ShotCalculator;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name = "🔥 AUTO HOOD (Camera Only)", group = "Combat")
public class CombatTeleOp_AutoHood extends OpMode {

    private Follower follower;
    private SimpleTurret turret;
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

    private Pose resetPose = new Pose(8, 8, 0);
    private boolean isRedAlliance = true;
    private int targetTagId = 24;
    private double targetX = RED_BASKET_X, targetY = RED_BASKET_Y;

    // Память для худа (если потеряли тег, держим последнее значение)
    private double lastAutoHoodPos = 0.35;

    // Toggle X Logic
    private boolean manualMode = false;
    private boolean lastX = false;
    private boolean cameraSetup = false;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(8, 8, 0));

        turret = new SimpleTurret(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        vision = new Vision(hardwareMap);
        hood = new Hood(hardwareMap);
        claw = new Claw(hardwareMap);

        // Просто создаем, никаких координат ему больше не нужно
        shotCalculator = new ShotCalculator();

        claw.close();
        telemetry.addLine("WAITING FOR CAMERA...");
        telemetry.addLine("SELECT: [B]=RED, [X]=BLUE");
    }

    @Override
    public void init_loop() {
        if (!cameraSetup) cameraSetup = vision.applyCombatSettings();

        if (gamepad1.b) {
            isRedAlliance = true;
            targetTagId = 24;
            targetX = RED_BASKET_X; targetY = RED_BASKET_Y;
            resetPose = new Pose(8, 8, 0);
            follower.setStartingPose(resetPose);
        }
        if (gamepad1.x) {
            isRedAlliance = false;
            targetTagId = 20;
            targetX = BLUE_BASKET_X; targetY = BLUE_BASKET_Y;
            resetPose = new Pose(136, 8, 0);
            follower.setStartingPose(resetPose);
        }

        if (cameraSetup) telemetry.addData("Vision", "✅ READY");
        telemetry.addData("Alliance", isRedAlliance ? "RED" : "BLUE");
        telemetry.update();
    }

    @Override
    public void start() { follower.startTeleopDrive(); }

    @Override
    public void loop() {
        follower.update();
        shooter.update();

        Pose pose = follower.getPose();
        AprilTagDetection tag = vision.getTarget(targetTagId);

        if (gamepad1.options) follower.setPose(resetPose);

        // Toggle X
        boolean currentX = gamepad2.x;
        if (currentX && !lastX) manualMode = !manualMode;
        lastX = currentX;

        boolean aim = gamepad2.left_trigger > 0.1;
        boolean fire = gamepad2.right_trigger > 0.1;

        // Для башни координаты все еще нужны (она сама их считает)
        turret.setTargetCoordinate(targetX, targetY);

        // === РАСЧЕТ ХУДА (НОВАЯ ЛОГИКА) ===
        if (tag != null) {
            // 1. Берем дистанцию с камеры
            double cameraDist = tag.ftcPose.range;

            // 2. Скармливаем её калькулятору
            lastAutoHoodPos = shotCalculator.calculateHoodPos(cameraDist);
        }
        // Если тега нет (tag == null), lastAutoHoodPos просто не меняется

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
                hood.update(shooter.getCurrentRPM(), 0);
                if (!aim) currentState = RobotState.IDLE;
                else if (fire && shooter.isReady()) currentState = RobotState.SHOOTING;
                break;

            case SHOOTING:
                shooter.setTargetRPM(3500);

                if (manualMode) {
                    // STATIC MODE (По кнопке X)
                    hood.setBasePosition(0.35);
                } else {
                    // AUTO MODE (Берем рассчитанное значение)
                    hood.setBasePosition(lastAutoHoodPos);
                }

                hood.update(shooter.getCurrentRPM(), 3500);
                claw.open(); intake.intake();

                if (!aim || !fire) currentState = aim ? RobotState.PREP_SHOOT : RobotState.IDLE;
                break;
        }

        // Turret Logic
        if (manualMode) {
            turret.setTargetAngle(0);
        }
        else if (currentState == RobotState.PREP_SHOOT || currentState == RobotState.SHOOTING) {
            turret.track(targetTagId, targetX, targetY);
        }
        else {
            turret.idle();
        }

        turret.update(pose, tag);
        handleChassis();

        telemetry.addData("MODE", manualMode ? "⚠️ STATIC" : "🤖 AUTO (Cam Only)");
        telemetry.addData("Hood Target", "%.3f", manualMode ? 0.35 : lastAutoHoodPos);
        telemetry.addData("Vision", tag != null ? "LOCKED" : "NO TAG (Using Memory)");

        telemetry.update();
    }

    private void stopAll() { shooter.setTargetRPM(0); intake.stop(); claw.close(); }

    private void handleChassis() {
        double drive = Math.pow(-gamepad1.left_stick_y, 3);
        double strafe = Math.pow(-gamepad1.left_stick_x, 3);
        double turn = Math.pow(gamepad1.left_trigger - gamepad1.right_trigger, 3);
        if (gamepad1.right_bumper) { drive *= 0.3; strafe *= 0.3; turn *= 0.3; }
        if (gamepad1.b) { drive = 0; strafe = 0; turn = 0; }
        follower.setTeleOpDrive(drive, strafe, turn, false);
    }
}