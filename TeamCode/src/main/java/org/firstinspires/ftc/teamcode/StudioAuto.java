package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/** @noinspection ALL*/
@Autonomous(name="StudioAuto", group="Linear Opmode")
public class StudioAuto extends LinearOpMode {

    private DcMotor leftFront, leftBack, rightBack, rightFront;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private StudioOdometry odo;
    private com.qualcomm.robotcore.hardware.Servo trapDoor;
    private com.qualcomm.robotcore.hardware.Servo flywheel;
    private com.qualcomm.robotcore.hardware.TouchSensor ballSensor;

    @Override
    public void runOpMode() throws InterruptedException {
        // --- Initialize drivetrain ---
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        trapDoor = hardwareMap.get(com.qualcomm.robotcore.hardware.Servo.class, "trapDoor");
        flywheel = hardwareMap.get(com.qualcomm.robotcore.hardware.Servo.class, "flywheel");
        ballSensor = hardwareMap.get(com.qualcomm.robotcore.hardware.TouchSensor.class, "ballSensor");
        odo = new StudioOdometry(hardwareMap);

        // --- Setup AprilTag processor ---
        aprilTag = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        telemetry.addLine("Initialized. Waiting for start...");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // Step 1: Drive forward ~50 inches
            driveForward(50.0, 0.5);

            // Step 2: Detect AprilTag
            int tagID = detectTagID();

            telemetry.addData("Tag ID", tagID);
            if (tagID == 21) {
                telemetry.addLine("Confirmed pattern: GPP");
            } else if (tagID == 22) {
                telemetry.addLine("Confirmed pattern: PGP");
            } else if (tagID == 33) {
                telemetry.addLine("Confirmed pattern: PPG");
            } else {
                telemetry.addLine("Confirmed pattern: None");
            }
            telemetry.update();

            // Step 3: Branch based on ID
            if (tagID == 21) {
                runScenarioGPP();
            } else if (tagID == 22) {
                runScenarioPGP();
            } else if (tagID == 33) {
                runScenarioPPG();
            } else {
                telemetry.addLine("No valid tag detected, running fallback auto.");
                telemetry.update();
                runFallback();
            }
        }
    }

    // --- Drive helper (very simplified; replace with encoder logic) ---
    private void driveForward(double inches, double power) {
        // TODO: implement encoder-based drive to move exact distance
        leftFront.setPower(power);
        leftBack.setPower(power);
        rightFront.setPower(power);
        rightBack.setPower(power);
        sleep((long)(inches * 40));  // rough timing placeholder
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }

    // --- AprilTag detection ---
    private int detectTagID() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        if (!detections.isEmpty()) {
            return detections.get(0).id;
        }
        return -1;  // no tag found
    }

    // --- Auto routines ---
    private void runScenarioGPP() {
        telemetry.addLine("Running runScenarioGPP");
        telemetry.addLine("Scenario GPP detected. Executing path...");
        telemetry.update();
        // Move to PPG, then cycle twice, then launch 3 balls
        goToPattern("PPG");
        performCycle();
        performCycle();
        goToGoalPosition();
        performLaunchSequence(3);
        // Back and turn then go to PGP, then launch 3 balls
        goBackAndTurn();
        goToPattern("PGP");
        goToGoalPosition();
        performLaunchSequence(3);
    }

    private void runScenarioPPG() {
        telemetry.addLine("Running runScenarioPPG");
        telemetry.addLine("Scenario PPG detected. Executing path...");
        telemetry.update();
        // Go to PPG and launch 3 balls
        goToPattern("PPG");
        goToGoalPosition();
        performLaunchSequence(3);
        // Back and turn then go to PGP, then cycle then launch 3 balls
        goBackAndTurn();
        goToPattern("PGP");
        performCycle();
        goToGoalPosition();
        performLaunchSequence(3);
    }

    private void runScenarioPGP() {
        telemetry.addLine("Running runScenarioPGP");
        telemetry.addLine("Scenario PGP detected. Executing path...");
        telemetry.update();
        // Go to GPP then launch 3 balls
        goToPattern("GPP");
        goToGoalPosition();
        performLaunchSequence(3);
        // Back and turn then go to PGP and launch 3 balls
        goBackAndTurn();
        goToPattern("PGP");
        goToGoalPosition();
        performLaunchSequence(3);
    }

    private void runFallback() {
        telemetry.addLine("Running runFallback");
        telemetry.addLine("Running safe backup routine...");
        telemetry.update();
        // TODO: e.g., park in safe zone
    }

    private void performCycle() {
        telemetry.addLine("Running performCycle");
        telemetry.addLine("Cycling ball...");
        telemetry.update();
        trapDoor.setPosition(1.0);
        sleep(500);
        trapDoor.setPosition(0.0);
        sleep(500);
    }

    private void performLaunchSequence(int count) {
        telemetry.addLine("Running performLaunchSequence");
        for (int i = 0; i < count; i++) {
            waitForBall();
            telemetry.addLine("Launching ball " + (i+1));
            telemetry.update();
            flywheel.setPosition(1.0);
            sleep(500);
            flywheel.setPosition(0.0);
            sleep(500);
        }
    }

    private void waitForBall() {
        while (opModeIsActive() && !ballSensor.isPressed()) {
            sleep(10);
        }
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

    private void driveForwardOdo(double targetInches, double power) {
        double startY = odo.getY();
        double targetY = startY + targetInches;

        // Reset odometry before moving
        odo.reset();

        while (opModeIsActive() && (odo.getY() < targetY)) {
            // Drive motors forward
            leftFront.setPower(power);
            leftBack.setPower(power);
            rightFront.setPower(power);
            rightBack.setPower(power);

            // Continuously update odometry
            odo.update();
            telemetry.addData("Y", odo.getY());
            telemetry.addData("leftFront Encoder", leftFront.getCurrentPosition());
            telemetry.addData("leftBack Encoder", leftBack.getCurrentPosition());
            telemetry.addData("rightFront Encoder", rightFront.getCurrentPosition());
            telemetry.addData("rightBack Encoder", rightBack.getCurrentPosition());
            telemetry.update();
        }

        // Stop motors
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }

    private void turnToHeadingOdo(double targetHeading, double power) {
        // Normalize target to [0, 360)
        targetHeading = (targetHeading % 360 + 360) % 360;

        while (opModeIsActive()) {
            odo.update();
            double currentHeading = odo.getHeadingDegrees();
            double error = targetHeading - currentHeading;

            if (Math.abs(error) < 2.0) break; // within 2 degrees

            double turnPower = Math.copySign(power, error);
            leftFront.setPower(-turnPower);
            leftBack.setPower(-turnPower);
            rightFront.setPower(turnPower);
            rightBack.setPower(turnPower);

            telemetry.addData("Heading", currentHeading);
            telemetry.addData("Target", targetHeading);
            telemetry.addData("leftFront Encoder", leftFront.getCurrentPosition());
            telemetry.addData("leftBack Encoder", leftBack.getCurrentPosition());
            telemetry.addData("rightFront Encoder", rightFront.getCurrentPosition());
            telemetry.addData("rightBack Encoder", rightBack.getCurrentPosition());
            telemetry.update();
        }

        // Stop
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }

    private void goTo(double targetX, double targetY, double power) {
        final double tolerance = 1.0; // inches
        final double kP = 0.1; // proportional constant for motor power

        odo.update();

        while (opModeIsActive()) {
            odo.update();
            double currentX = odo.getX();
            double currentY = odo.getY();

            double errorX = targetX - currentX;
            double errorY = targetY - currentY;

            if (Math.abs(errorX) < tolerance && Math.abs(errorY) < tolerance) {
                break;
            }

            // Calculate motor powers based on errors (simple proportional control)
            double powerX = kP * errorX;
            double powerY = kP * errorY;

            // Clamp powers to [-power, power]
            powerX = Math.max(-power, Math.min(powerX, power));
            powerY = Math.max(-power, Math.min(powerY, power));

            // For a mecanum or holonomic drive, set powers accordingly:
            // Here, assume simple tank drive, so combine X and Y powers for forward and strafe
            // But since the original code is tank drive, we simplify to forward/backward power only
            // So we use powerY for forward/backward and ignore powerX (or could implement turning)

            // Set motor powers to move forward/backward based on Y error
            leftFront.setPower(powerY);
            leftBack.setPower(powerY);
            rightFront.setPower(powerY);
            rightBack.setPower(powerY);

            telemetry.addData("Current X", currentX);
            telemetry.addData("Current Y", currentY);
            telemetry.addData("Target X", targetX);
            telemetry.addData("Target Y", targetY);
            telemetry.update();

            sleep(20);
        }

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }
}