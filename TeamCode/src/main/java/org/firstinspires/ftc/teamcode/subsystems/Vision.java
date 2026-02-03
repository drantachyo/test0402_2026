package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Size;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

public class Vision {
    private AprilTagProcessor aprilTag;
    private VisionPortal portal;

    public Vision(HardwareMap hw) {
        aprilTag = new AprilTagProcessor.Builder()
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .build();

        portal = new VisionPortal.Builder()
                .setCamera(hw.get(WebcamName.class, "Webcam"))
                .addProcessor(aprilTag)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();
    }

    /**
     * === БОЕВЫЕ НАСТРОЙКИ (ТВОИ 2ms / 240 Gain) ===
     * Вызывай этот метод во всех TeleOp в init_loop()
     */
    public boolean applyCombatSettings() {
        // Твои жесткие значения:
        return setManualExposure(2, 240);
    }

    // Внутренняя логика установки (не меняй)
    public boolean setManualExposure(int exposureMS, int gain) {
        if (portal == null || portal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            return false;
        }

        // Если настройки уже стоят, не спамим командами (экономим время цикла)
        // Но для надежности отправляем, если это первый раз
        try {
            ExposureControl exposureControl = portal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                Thread.sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            Thread.sleep(20);

            GainControl gainControl = portal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            Thread.sleep(20);
            return true;
        } catch (InterruptedException e) {
            return false;
        }
    }

    public AprilTagDetection getTarget(int id) {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        if (detections == null) return null;
        for (AprilTagDetection d : detections) {
            if (d.id == id && d.metadata != null) return d;
        }
        return null;
    }

    public AprilTagDetection getAnyTarget() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        if (detections == null) return null;
        for (AprilTagDetection d : detections) {
            if (d.metadata != null) return d;
        }
        return null;
    }

    public void stop() {
        if (portal != null) portal.close();
    }
}