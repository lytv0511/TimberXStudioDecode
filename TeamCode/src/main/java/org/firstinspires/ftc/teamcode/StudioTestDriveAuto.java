package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@Autonomous(name = "StudioTestDriveAuto", group = "Studio")
public class StudioTestDriveAuto extends LinearOpMode {
    // Declare motors
    private DcMotor leftFront, rightFront, leftBack, rightBack;
    // Odometry and AprilTag
    private StudioOdometry odometry;
    private StudioAprilTag studioAprilTag;

    // Constants
    private static final double INCHES_TO_TICKS = 1000.0 / 86.0; // fudge factor: 1000 ticks = 86 inches (example)
    private static final double DRIVE_KP = 0.02; // proportional constant for drive (more aggressive)
    private static final double STRAFE_KP = 0.02;
    private static final double TURN_KP = 0.005;
    private static final double MAX_FORWARD_POWER = 1.0;
    private static final double MAX_STRAFE_POWER = 0.8;
    private static final double TURN_MAX_POWER = 0.5;
    private static final double HEADING_TOLERANCE = 2.0; // degrees
    private static final double POSITION_TOLERANCE = 2.5; // inches

    @Override
    public void runOpMode() {
        // Hardware
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        setAllMotorsZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Odometry and AprilTag
        odometry = new StudioOdometry(hardwareMap);
        studioAprilTag = new StudioAprilTag();
        studioAprilTag.init(hardwareMap, "Webcam 1");

        telemetry.addLine("Ready. Waiting for start...");
        telemetry.update();
        waitForStart();

        // Reset odometry
        odometry.reset();

        // 1. Move forward 86 inches
//        moveForwardInches(86.0);
        strafeRightInches(-86.0);
        rotateDegrees(270);
        // Do NOT stop all motors here before detection unless reached target

        // 2. Detect tag using polling loop for up to 2.5 seconds
        telemetry.addLine("Detecting AprilTag...");
        telemetry.update();
        int tagID = -1;
        long detectStart = System.currentTimeMillis();
        while (opModeIsActive() && (System.currentTimeMillis() - detectStart < 2500) && tagID == -1) {
            List<AprilTagDetection> detections = studioAprilTag.getDetections();
            if (!detections.isEmpty()) {
                tagID = detections.get(0).id;
            } else {
                telemetry.addLine("Detecting AprilTag...");
                telemetry.addData("Elapsed (ms)", System.currentTimeMillis() - detectStart);
                telemetry.update();
                sleep(50);
            }
        }
        telemetry.addData("AprilTag ID", tagID);
        telemetry.update();

        // 3. Decision logic
        if (tagID == 21) {
            // Move back 86 inches to original spot
            runGPP();
        } else if (tagID == 22) {
            // Move right 30 inches
            runPGP();
        } else if (tagID == 23) {
            runPPG();
        } else {
            telemetry.addLine("No valid tag detected, doing nothing.");
            telemetry.update();
        }
        stopAllMotors();
        telemetry.addLine("Done!");
        telemetry.update();
        sleep(2000);

        studioAprilTag.shutdown();
    }

    private void setAllMotorsZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        leftFront.setZeroPowerBehavior(behavior);
        rightFront.setZeroPowerBehavior(behavior);
        leftBack.setZeroPowerBehavior(behavior);
        rightBack.setZeroPowerBehavior(behavior);
    }

    private void stopAllMotors() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }

    // Move forward/backward using odometry (positive = forward, negative = backward)
    private void moveForwardInches(double inches) {
        double startY = odometry.getY();
        double startHeading = odometry.getHeadingDegrees();
        double targetY = startY + inches;
        long startTime = System.currentTimeMillis();
        while (opModeIsActive() && Math.abs(odometry.getY() - targetY) > POSITION_TOLERANCE && System.currentTimeMillis() - startTime < 3000) {
            odometry.update();
            double error = targetY - odometry.getY();
            double distanceRemaining = Math.abs(error);
            // Smooth ramping, start and end slow, max mid-way
            double rampDownDist = 20.0; // inches to start slowing down
            double minRampPower = 0.3;
            double rampScale = 1.0;
            if (distanceRemaining < rampDownDist) {
                rampScale = Math.max(minRampPower, distanceRemaining / rampDownDist);
            }
            double forward = Math.copySign(rampScale * MAX_FORWARD_POWER, error);
            // Clamp to max/min
            forward = Math.max(-MAX_FORWARD_POWER, Math.min(MAX_FORWARD_POWER, forward));

            // Heading correction applied as strafe to reduce drift
            double currentHeading = odometry.getHeadingDegrees();
            double headingError = currentHeading - startHeading;
            // Normalize heading error to [-180,180]
            while (headingError > 180) headingError -= 360;
            while (headingError < -180) headingError += 360;

            double strafeCorrection = -TURN_KP * headingError; // negative to counter drift
            // Clamp strafeCorrection to max strafe power limits
            strafeCorrection = Math.max(-MAX_STRAFE_POWER, Math.min(MAX_STRAFE_POWER, strafeCorrection));

            setDrivePowerFromInputs(forward, strafeCorrection, 0);
            telemetry.addData("Target Y", targetY);
            telemetry.addData("Current Y", odometry.getY());
            telemetry.addData("Forward", forward);
            telemetry.addData("Heading Error", headingError);
            telemetry.addData("Strafe Correction", strafeCorrection);
            telemetry.update();
        }
    }

    // Strafe right using odometry (positive inches = right, negative = left)
    private void strafeRightInches(double inches) {
        double startX = odometry.getX();
        double startY = odometry.getY();
        double startHeading = odometry.getHeadingDegrees();
        double targetX = startX + inches;
        long startTime = System.currentTimeMillis();
        while (opModeIsActive() && Math.abs(odometry.getX() - targetX) > POSITION_TOLERANCE && System.currentTimeMillis() - startTime < 3000) {
            odometry.update();
            double errorX = targetX - odometry.getX();
            double distanceRemaining = Math.abs(errorX);
            // Smooth ramping for strafe
            double rampDownDist = 20.0; // inches to start slowing down
            double minRampPower = 0.3;
            double rampScale = 1.0;
            if (distanceRemaining < rampDownDist) {
                rampScale = Math.max(minRampPower, distanceRemaining / rampDownDist);
            }
            double strafe = Math.copySign(rampScale * MAX_STRAFE_POWER, errorX);
            // Clamp to max/min
            strafe = Math.max(-MAX_STRAFE_POWER, Math.min(MAX_STRAFE_POWER, strafe));

            // Forward correction for Y drift
            double forwardCorrection = DRIVE_KP * (startY - odometry.getY());

            // Heading correction for turn if heading deviates beyond tolerance
            double currentHeading = odometry.getHeadingDegrees();
            double headingError = currentHeading - startHeading;
            // Normalize heading error to [-180,180]
            while (headingError > 180) headingError -= 360;
            while (headingError < -180) headingError += 360;

            double turnCorrection = 0;
            if (Math.abs(headingError) > HEADING_TOLERANCE) {
                turnCorrection = TURN_KP * headingError;
                // Clamp turnCorrection to max turn power limits
                turnCorrection = Math.max(-TURN_MAX_POWER, Math.min(TURN_MAX_POWER, turnCorrection));
            }

            setDrivePowerFromInputs(forwardCorrection, strafe, turnCorrection);
            telemetry.addData("Target X", targetX);
            telemetry.addData("Current X", odometry.getX());
            telemetry.addData("Strafe", strafe);
            telemetry.addData("Forward Correction", forwardCorrection);
            telemetry.addData("Heading Error", headingError);
            telemetry.addData("Turn Correction", turnCorrection);
            telemetry.update();
        }
    }

    // Rotate in place by degrees (positive = clockwise)
    private void rotateDegrees(double degrees) {
        double startHeading = odometry.getHeadingDegrees();
        double targetHeading = startHeading + degrees;
        double minTurnPower = 0.25; // minimum power to overcome motor deadband
        double maxTurnPower = 0.6;
        double rampDownStart = 20.0;

        boolean crossedZero = false;
        double prevError = angleDiff(odometry.getHeadingDegrees(), targetHeading);

        while (opModeIsActive()) {
            odometry.update();
            double currentHeading = odometry.getHeadingDegrees();
            double error = angleDiff(currentHeading, targetHeading);
            double absError = Math.abs(error);

            if (absError <= HEADING_TOLERANCE) {
                break;
            }

            // Detect sign change crossing zero when approaching target heading
            if (Math.signum(error) != Math.signum(prevError)) {
                if (crossedZero) {
                    // Already corrected once, exit loop
                    break;
                } else {
                    crossedZero = true;
                }
            }
            prevError = error;

            double powerScale;
            if (absError > rampDownStart) {
                powerScale = 1.0;
            } else {
                powerScale = minTurnPower + (maxTurnPower - minTurnPower) * (absError / rampDownStart);
                if (powerScale > maxTurnPower) {
                    powerScale = maxTurnPower;
                } else if (powerScale < minTurnPower) {
                    powerScale = minTurnPower;
                }
                powerScale /= maxTurnPower;
            }

            double turnPower = powerScale * maxTurnPower * Math.signum(error);
            if (Math.abs(turnPower) < minTurnPower) {
                turnPower = minTurnPower * Math.signum(turnPower);
            }

            setDrivePowerFromInputs(0, 0, turnPower);

            telemetry.addData("Target Heading", targetHeading);
            telemetry.addData("Current Heading", currentHeading);
            telemetry.addData("Heading Error", error);
            telemetry.addData("Turn Power", turnPower);
            telemetry.update();
        }
    }

    // Helper for drive power: [LF, RF, LR, RR]
    private void setDrivePower(double lf, double rf, double lr, double rr) {
        leftFront.setPower(lf);
        rightFront.setPower(rf);
        leftBack.setPower(lr);
        rightBack.setPower(rr);
    }

    // Helper for angle difference (-180 to 180)
    private double angleDiff(double a, double b) {
        double diff = b - a;
        while (diff > 180) diff -= 360;
        while (diff < -180) diff += 360;
        return diff;
    }

    /**
     * Polls AprilTag detections for up to 2.5 seconds, returns the first detected tag's ID, or -1 if none.
     */
    private int detectTagID() {
        List<AprilTagDetection> detections = studioAprilTag.getDetections();
        if (!detections.isEmpty()) {
            return detections.get(0).id;
        }
        return -1;
    }

    /**
     * Uses the motor power calculation logic from StudioTeleop.java
     * Inputs are forward, strafe, turn power components.
     * Forward: -1.0 to 1.0 (MAX_FORWARD_POWER)
     * Strafe: -1.0 to 1.0, but scaled to MAX_STRAFE_POWER
     * Turn: -1.0 to 1.0, but scaled to TURN_MAX_POWER
     */
    private void setDrivePowerFromInputs(double forward, double strafe, double turn) {
        // Clamp input to [-1, 1]
        forward = Math.max(-1.0, Math.min(1.0, forward));
        strafe  = Math.max(-1.0, Math.min(1.0, strafe));
        turn    = Math.max(-1.0, Math.min(1.0, turn));

        double forwardScaled = forward * MAX_FORWARD_POWER;
        double strafeScaled = strafe * MAX_STRAFE_POWER;
        double turnScaled = turn * TURN_MAX_POWER;

        double lf = forwardScaled + strafeScaled + turnScaled;
        double rf = -forwardScaled + strafeScaled + turnScaled;
        double lr = forwardScaled - strafeScaled + turnScaled;
        double rr = -forwardScaled - strafeScaled + turnScaled;

        // Find the maximum absolute value among the powers
        double max = Math.max(Math.abs(lf), Math.abs(rf));
        max = Math.max(max, Math.abs(lr));
        max = Math.max(max, Math.abs(rr));

        // Normalize powers if any is outside the range [-1, 1]
        if (max > 1.0) {
            lf /= max;
            rf /= max;
            lr /= max;
            rr /= max;
        }

        setDrivePower(lf, rf, lr, rr);
    }

    private void runGPP() {
        moveForwardInches(30);
        rotateDegrees(90);
        moveForwardInches(86);
        moveForwardInches(-30);
    }

    private void runPGP(){
        rotateDegrees(180);
        strafeRightInches(-86);
        rotateDegrees(360);
    }

    private void runPPG() {
        rotateDegrees(360);
        strafeRightInches(86);
    }
}