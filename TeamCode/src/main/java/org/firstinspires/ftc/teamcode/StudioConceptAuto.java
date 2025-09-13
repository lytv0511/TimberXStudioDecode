package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@Autonomous(name="StudioConceptAuto", group="Linear Opmode")
public class StudioConceptAuto extends LinearOpMode {

    private DcMotor leftFront, leftBack, rightBack, rightFront;
    private StudioAprilTag studioAprilTag;

    @Override
    public void runOpMode() throws InterruptedException {
        // --- Initialize drivetrain ---
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        studioAprilTag = new StudioAprilTag();
        studioAprilTag.init(hardwareMap, "Webcam 1");

        telemetry.addLine("Initialized. Waiting for start...");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            int tagId = -1;
            driveForward(0.5, 1000);
            while (opModeIsActive() && tagId == -1) {
                tagId = detectTagID();
                if (tagId == -1) {
                    telemetry.addLine("No AprilTag detected, waiting...");
                    telemetry.update();
                    sleep(100);
                }
            }

            // Telemetry for detected tag ID and pattern
            telemetry.addLine("Scanned result:");
            telemetry.addData("Detected Tag ID", tagId);
            if (tagId == 21) {
                telemetry.addData("Pattern", "GPP");
            } else if (tagId == 22) {
                telemetry.addData("Pattern", "PGP");
            } else if (tagId == 23) {
                telemetry.addData("Pattern", "PPG");
            } else {
                telemetry.addData("Pattern", "Unknown");
            }
            telemetry.update();

            // Execute corresponding command based on tagId
            if (tagId == 22) { // PGP: spin clockwise then anticlockwise
                turn(0.5, 500);
                turn(-0.5, 500);
            } else if (tagId == 23) { // PPG: spin clockwise
                turn(0.5, 5000);
            } else if (tagId == 21) { // GPP: spin anticlockwise
                turn(0.5, 500);
                turn(-0.5, 500);
            } else {
                runFallback();
            }

            turn(0.5, 2000);
            driveForward(0.5, 1000);
            tagId = -1;
            while (opModeIsActive() && tagId == -1) {
                tagId = detectTagID();
                if (tagId == -1) {
                    telemetry.addLine("No AprilTag detected, waiting...");
                    telemetry.update();
                    sleep(100);
                }
            }

            // Telemetry for detected tag ID and pattern
            telemetry.addLine("Scanned result:");
            telemetry.addData("Detected Tag ID", tagId);
            if (tagId == 21) {
                telemetry.addData("Pattern", "GPP");
            } else if (tagId == 22) {
                telemetry.addData("Pattern", "PGP");
            } else if (tagId == 23) {
                telemetry.addData("Pattern", "PPG");
            } else {
                telemetry.addData("Pattern", "Unknown");
            }
            telemetry.update();

            // Execute corresponding command based on tagId
            if (tagId == 22) { // PGP: spin clockwise then anticlockwise
                turn(0.5, 5000);
                turn(-0.5, 5000);
            } else if (tagId == 23) { // PPG: spin clockwise
                turn(0.5, 5000);
            } else if (tagId == 21) { // GPP: spin anticlockwise
                turn(-0.5, 5000);
            } else {
                runFallback();
            }

            studioAprilTag.shutdown();
        }
    }

    private int detectTagID() {
        List<AprilTagDetection> detections = studioAprilTag.getDetections();
        if (!detections.isEmpty()) {
            return detections.get(0).id;
        }
        return -1;
    }

    // --- Drive helper (very simplified; replace with encoder logic) ---
    private void driveForward(double power, long milliseconds) {
        // Set motor powers to drive forward
        leftFront.setPower(power);
        leftBack.setPower(power);
        rightFront.setPower(-power); // Reverse direction for right side
        rightBack.setPower(power);

        // Wait for the specified time
        sleep(milliseconds);

        // Stop all motors
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }

    private void driveSideways(double power, long milliseconds) {
        // Strafe right (positive power) or left (negative power)
        leftFront.setPower(power);
        leftBack.setPower(-power);
        rightFront.setPower(power);
        rightBack.setPower(power);

        sleep(milliseconds);

        // Stop all motors
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }

    private void turn(double power, long milliseconds) {
        // Rotate in place
        leftFront.setPower(power);
        leftBack.setPower(power);
        rightFront.setPower(power);
        rightBack.setPower(-power);

        sleep(milliseconds);

        // Stop all motors
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }

    private void runFallback() {
        telemetry.addLine("Running runFallback");
        telemetry.addLine("Running safe backup routine...");
        telemetry.update();
        // TODO: e.g., park in safe zone
    }

    private void goToPattern(String patternName) {
        telemetry.addLine("Running goToPattern");
        telemetry.addData("Moving to pattern", patternName);
        telemetry.update();
        // Placeholder for movement to pattern location
        sleep(1000);
    }

    private void goBackAndTurn() {
        telemetry.addLine("Running goBackAndTurn");
        telemetry.addLine("Going back and turning...");
        telemetry.update();
        // Placeholder for moving back and turning
        sleep(1000);
    }

    private void goToGoalPosition() {
        telemetry.addLine("Running goToGoalPosition");
        telemetry.addLine("Moving to goal scoring position...");
        telemetry.update();
        // Placeholder for movement to the goal scoring position
        sleep(1000);
    }
}