package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * Wrapper around the AprilTag processor and VisionPortal.
 * Handles initialization, shutdown, and provides helpers to fetch detections.
 */
public class StudioAprilTag {

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    /**
     * Initialize AprilTag detection.
     * @param hwMap the hardware map
     * @param cameraName the name of the camera in the robot configuration (e.g., "Webcam 1")
     */
    public void init(HardwareMap hwMap, String cameraName) {
        // Camera pose relative to the robot center (tune these values for your bot!)
        Position cameraPosition = new Position(DistanceUnit.INCH, 0, 0, 6, 0);
        YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90, 0, 0);

        aprilTag = new AprilTagProcessor.Builder()
                .setCameraPose(cameraPosition, cameraOrientation)
                // .setDrawAxes(true)
                // .setDrawTagOutline(true)
                // .setTagFamily("tag36h11") // default family
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hwMap.get(WebcamName.class, cameraName));
        builder.addProcessor(aprilTag);
        visionPortal = builder.build();
        visionPortal.resumeStreaming();
    }

    /**
     * Get the latest AprilTag detections.
     * @return a list of detections (possibly empty, never null)
     */
    public List<AprilTagDetection> getDetections() {
        if (aprilTag != null) {
            return aprilTag.getDetections();
        }
        return java.util.Collections.emptyList();
    }

    /**
     * Choose a "best" detection from the list (e.g., first with metadata, else first with pose).
     * @return best detection or null if none available
     */
    public AprilTagDetection getBestDetection() {
        List<AprilTagDetection> detections = getDetections();
        if (detections == null || detections.isEmpty()) return null;

        for (AprilTagDetection d : detections) {
            if (d.robotPose != null && d.metadata != null) {
                return d;
            }
        }
        // fallback: just return first with a pose
        for (AprilTagDetection d : detections) {
            if (d.robotPose != null) {
                return d;
            }
        }
        return null;
    }

    /**
     * Stop streaming and free resources. Call at the end of your opmode.
     */
    public void shutdown() {
        if (visionPortal != null) {
            visionPortal.close();
            visionPortal = null;
        }
        aprilTag = null;
    }
}