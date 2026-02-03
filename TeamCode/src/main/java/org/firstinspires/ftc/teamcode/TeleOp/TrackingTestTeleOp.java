package org.firstinspires.ftc.teamcode.TeleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.SimpleTurret;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Configurable
@TeleOp(name = "🎯 TRACKING TEST (Combat Vision)", group = "Test")
public class TrackingTestTeleOp extends OpMode {

    private Follower follower;
    private SimpleTurret turret;
    private Vision vision;

    public static double TARGET_X = 0;
    public static double TARGET_Y = 70;

    // 1. Флаг для камеры
    boolean cameraSetup = false;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(8, 8, 0));

        turret = new SimpleTurret(hardwareMap);
        vision = new Vision(hardwareMap);

        turret.setActive(true);
        telemetry.addLine("Waiting for camera...");
    }

    // 2. ВОТ ЭТОТ КУСОК НУЖНО ВСТАВИТЬ ВО ВСЕ ТЕЛЕОПЫ
    @Override
    public void init_loop() {
        if (!cameraSetup) {
            // Пытаемся применить настройки 2ms/240
            cameraSetup = vision.applyCombatSettings();

            if (cameraSetup) {
                telemetry.addData("Vision", "✅ COMBAT MODE (2ms/240)");
            } else {
                telemetry.addData("Vision", "⏳ Setting Camera...");
            }
        }
        telemetry.update();
    }
    // ===========================================

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.update();
        Pose pose = follower.getPose();

        turret.setTargetCoordinate(TARGET_X, TARGET_Y);
        AprilTagDetection tag = vision.getAnyTarget();
        turret.update(pose, tag);

        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, gamepad1.left_trigger - gamepad1.right_trigger, false);

        telemetry.addData("Tag Visible", tag != null ? "YES" : "NO");
        telemetry.addData("Target", "%.1f, %.1f", TARGET_X, TARGET_Y);
        telemetry.update();
    }
}