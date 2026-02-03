package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.math.MathUtils;
import org.firstinspires.ftc.teamcode.math.PIDFController;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Configurable
public class SimpleTurret {
    public enum State { IDLE, SEARCHING_ODO, LOCKED_VISION, MANUAL }

    private final DcMotorEx motor;
    private final PIDFController controller;
    private State currentState = State.IDLE;

    // === НАСТРОЙКИ ===
    public static double TICKS_PER_RADIAN = 205.0;
    public static double LIMIT_MIN = Math.toRadians(-90);
    public static double LIMIT_MAX = Math.toRadians(90);

    // PID
    public static double ODOM_P = 2.5, ODOM_I = 0.0, ODOM_D = 0.08;
    public static double VIS_P = 1, VIS_I = 0.0, VIS_D = 0.06;

    private double targetX = 0, targetY = 0;
    private double manualTargetAngle = 0;
    private boolean isActive = false;

    public SimpleTurret(HardwareMap hw) {
        motor = hw.get(DcMotorEx.class, "turret");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        controller = new PIDFController(ODOM_P, ODOM_I, ODOM_D, 0, 0);
    }

    public void setTargetAngle(double radians) {
        this.manualTargetAngle = radians;
        this.isActive = true;
        this.currentState = State.MANUAL;
    }

    public void track(int id, double x, double y) {
        this.targetX = x; this.targetY = y;
        this.isActive = true;
        if (currentState == State.MANUAL) currentState = State.SEARCHING_ODO;
    }

    // === ВОТ ЭТОГО МЕТОДА НЕ ХВАТАЛО ===
    public void idle() {
        this.isActive = false;
        this.currentState = State.IDLE;
        motor.setPower(0);
    }

    public void setTargetCoordinate(double x, double y) {
        this.targetX = x; this.targetY = y;
    }

    public void setActive(boolean active) { this.isActive = active; }

    public void update(Pose robotPose, AprilTagDetection tag) {
        double currentAngle = getPosition();
        double targetAngle = 0;

        if (!isActive) {
            currentState = State.IDLE;
        }
        else if (currentState != State.MANUAL) {
            if (tag != null) currentState = State.LOCKED_VISION;
            else currentState = State.SEARCHING_ODO;
        }

        switch (currentState) {
            case IDLE:
                controller.setPIDF(ODOM_P, ODOM_I, ODOM_D, 0, 0);
                targetAngle = 0;
                break;

            case MANUAL:
                controller.setPIDF(ODOM_P, ODOM_I, ODOM_D, 0, 0);
                targetAngle = manualTargetAngle;
                break;

            case SEARCHING_ODO:
                controller.setPIDF(ODOM_P, ODOM_I, ODOM_D, 0, 0);
                if (robotPose != null) {
                    double dx = targetX - robotPose.getX();
                    double dy = targetY - robotPose.getY();
                    double absAngle = Math.atan2(dy, dx);
                    double relAngle = MathUtils.normalizeAngle(absAngle - robotPose.getHeading());
                    targetAngle = -relAngle;
                }
                break;

            case LOCKED_VISION:
                controller.setPIDF(VIS_P, VIS_I, VIS_D, 0, 0);
                double bearing = Math.toRadians(tag.ftcPose.bearing);
                targetAngle = currentAngle + bearing;
                break;
        }

        targetAngle = Range.clip(targetAngle, LIMIT_MIN, LIMIT_MAX);
        double error = MathUtils.normalizeAngle(targetAngle - currentAngle);
        double power = controller.calculate(error);

        if (currentAngle >= LIMIT_MAX && power > 0) power = 0;
        if (currentAngle <= LIMIT_MIN && power < 0) power = 0;

        motor.setPower(power);
    }

    public double getPosition() { return motor.getCurrentPosition() / TICKS_PER_RADIAN; }
    public State getState() { return currentState; }
}