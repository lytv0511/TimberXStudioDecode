package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.StudioOdometry;
import org.firstinspires.ftc.teamcode.StudioOdometry;

/** @noinspection ALL */
@TeleOp(name="StudioTeleop")
public class StudioTeleop extends LinearOpMode {
    private DcMotor leftFront, leftBack, rightBack, rightFront;
//    private Servo clawServo, clawArmServo;

    private StudioOdometry odometry;

    private void turnToAngle(double targetDeltaDegrees, double power) {
        double startHeading = odometry.getHeadingDegrees();
        double targetHeading = (startHeading + targetDeltaDegrees) % 360;

        // Ensure positive normalization [0, 360)
        if (targetHeading < 0) targetHeading += 360;

        // Spin in the correct direction
        double direction = (targetDeltaDegrees > 0) ? 1.0 : -1.0;

        // Start turning
        leftFront.setPower(direction * power);
        leftBack.setPower(direction * power);
        rightFront.setPower(direction * power);
        rightBack.setPower(direction * power);

        // Keep looping until heading is close to target
        while (opModeIsActive() && Math.abs(odometry.getHeadingDegrees() - targetHeading) > 2.0) {
            odometry.update();  // refresh odometry continuously
            telemetry.addData("Turning", "Current: %.2f Target: %.2f",
                    odometry.getHeadingDegrees(), targetHeading);
            telemetry.update();
        }

        // Stop motors
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }

    private void turnToHeadingPID(double targetHeading, double maxPower) {
        // Normalize target to [0, 360)
        targetHeading = (targetHeading % 360 + 360) % 360;

        double kP = 0.015;   // proportional gain (tune)
        double kI = 0.0;     // integral gain (optional)
        double kD = 0.001;   // derivative gain (tune)

        double prevError = 0;
        double integral = 0;
        long lastTime = System.nanoTime();

        // Helper to compute signed minimal error [-180, 180]
        java.util.function.BiFunction<Double, Double, Double> getHeadingError = (target, current) -> {
            double error = target - current;
            if (error > 180) error -= 360;
            if (error < -180) error += 360;
            return error;
        };

        while (opModeIsActive()) {
            odometry.update();
            double current = odometry.getHeadingDegrees();

            // Compute minimal signed error
            double error = getHeadingError.apply(targetHeading, current);

            // Stop if within tolerance
            if (Math.abs(error) < 0.5) break;

            // Delta time in seconds
            long now = System.nanoTime();
            double dt = (now - lastTime) / 1e9;
            lastTime = now;

            // PID calculations
            integral += error * dt;
            double derivative = (error - prevError) / dt;

            // Clamp derivative to prevent spikes
            double maxDerivative = 10; // tuned for stability
            derivative = Math.max(-maxDerivative, Math.min(maxDerivative, derivative));

            prevError = error;

            double output = kP * error + kI * integral + kD * derivative;

            // Clamp output to max power
            output = Math.max(-maxPower, Math.min(maxPower, output));

            // Deadband to prevent twitching
            if (Math.abs(output) < 0.05) output = 0;

            // Apply turning power (all + output, as requested)
            leftFront.setPower(output);
            leftBack.setPower(output);
            rightFront.setPower(output);
            rightBack.setPower(output);

            telemetry.addData("PID Turn", "Current: %.2f Target: %.2f Error: %.2f Power: %.2f",
                    current, targetHeading, error, output);
            telemetry.update();
        }

        // Stop motors
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }

    @Override
    public void runOpMode() {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
//        clawServo = hardwareMap.get(Servo.class, "clawServo");
//        clawArmServo = hardwareMap.get(Servo.class, "clawArmServo");

        // --- Initialize Odometry ---
        odometry = new StudioOdometry(hardwareMap);

        odometry.reset();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        boolean isHoverMode = false;
        boolean isRetractMode = false;
        boolean isModeTwo = false;
        boolean leftStickButtonPressed = false;
        boolean isTurnInProgress = false;
        boolean holdSlidesMode = false;
        boolean isNormalGrabClosed = false;
        boolean isForceGrabClosed = false;
        boolean yButtonPressed = false;
        boolean xButtonPressed = false;

        while (opModeIsActive()) {
            odometry.update();

            // Toggle between Mode 1 and Mode 2 (Inverting Movement)
            if (gamepad1.left_stick_button && !leftStickButtonPressed) {
                isModeTwo = !isModeTwo;
                leftStickButtonPressed = true;
            }
            if (!gamepad1.left_stick_button) {
                leftStickButtonPressed = false;
            }

//            // Toggle Normal Grab (Y button)
//            if (gamepad1.y && !yButtonPressed) {
//                isNormalGrabClosed = !isNormalGrabClosed;
//                isForceGrabClosed = false;
//                clawServo.setPosition(isNormalGrabClosed ? 0.4 : 0.6);
//                yButtonPressed = true;
//            }
//            if (!gamepad1.y) {
//                yButtonPressed = false;
//            }
//
//            // Toggle Force Grab (X button)
//            if (gamepad1.x && !xButtonPressed) {
//                isForceGrabClosed = !isForceGrabClosed;
//                isNormalGrabClosed = false;
//                clawServo.setPosition(isForceGrabClosed ? 0.4 : 0.7);
//                xButtonPressed = true;
//            }
//            if (!gamepad1.x) {
//                xButtonPressed = false;
//            }

            boolean sweepingMode = false;
            boolean sweepingDirection = true;
            long lastSweepTime = System.currentTimeMillis();
            long sweepInterval = 500; // Adjust speed of sweeping motion

//            if (gamepad1.b) {
//                clawArmServo.setPosition(0.1);
//                clawServo.setPosition(0.4); // Open claw
//                sweepingMode = true;
//                telemetry.update();
//
//                while (gamepad1.b && opModeIsActive()) {
//                    double sweepPower = 0.5;
//
//                    // Sweep in one direction
//                    leftFront.setPower(sweepPower);
//                    leftBack.setPower(sweepPower);
//                    rightFront.setPower(sweepPower);
//                    rightBack.setPower(sweepPower);
//                    sleep(500);
//
//                    // Reverse sweeping direction
//                    leftFront.setPower(-sweepPower);
//                    leftBack.setPower(-sweepPower);
//                    rightFront.setPower(-sweepPower);
//                    rightBack.setPower(-sweepPower);
//                    sleep(500);
//                }
//
//                // Stop movement when B is released
//                leftFront.setPower(0);
//                rightFront.setPower(0);
//                leftBack.setPower(0);
//                rightBack.setPower(0);
//            }
//
//
//            // 180 Degree Turn (A button)
//            if (gamepad1.a && !isTurnInProgress) {
//                isTurnInProgress = true;
//                isModeTwo = !isModeTwo; // Invert movement mode
//                clawArmServo.setPosition(0.9);
//
//                turnToHeadingPID(180, 1);
//
//                isTurnInProgress = false;
//            }
//
////            if (gamepad1.dpad_up && !isTurnInProgress) {
////                isTurnInProgress = true;
////                turnToHeading(0.0, 0.5);   // face "forward"
////                isTurnInProgress = false;
////            }
////            if (gamepad1.dpad_right && !isTurnInProgress) {
////                isTurnInProgress = true;
////                turnToHeading(90.0, 0.5);  // face right
////                isTurnInProgress = false;
////            }
////            if (gamepad1.dpad_down && !isTurnInProgress) {
////                isTurnInProgress = true;
////                turnToHeading(180.0, 0.5); // face backward
////                isTurnInProgress = false;
////            }
////            if (gamepad1.dpad_left && !isTurnInProgress) {
////                isTurnInProgress = true;
////                turnToHeading(270.0, 0.5); // face left
////                isTurnInProgress = false;
////            }
//
//            boolean hanging = false;
//
//            // Claw Arm Hover & Retract (Triggers)
//            if (gamepad1.right_trigger > 0) {
//                isHoverMode = true;
//                clawArmServo.setPosition(0.2);
//            }
//            if (gamepad1.right_trigger == 0 && isHoverMode && !hanging) {
//                isHoverMode = false;
//                clawArmServo.setPosition(0.1);
//            }
//
//            if (gamepad1.left_trigger > 0) {
//                isRetractMode = true;
//                clawArmServo.setPosition(0.2);
//            }
//            if (gamepad1.left_trigger == 0 && isRetractMode) {
//                isRetractMode = false;
//                clawArmServo.setPosition(0.9);
//            }
//            if (gamepad1.right_bumper) {
//                hanging = true;
//                clawArmServo.setPosition(0.4);
//            } else if (!gamepad1.right_bumper) {
//                hanging = false;
//            }

            // Slides Control with D-Pad
//            if (gamepad1.dpad_up) {
//                linearLeft.setPower(-1);
//                linearRight.setPower(1);
//            } else if (gamepad1.dpad_down) {
//                linearLeft.setPower(1);
//                linearRight.setPower(-1);
//            } else if (gamepad1.dpad_right) {
//                holdSlidesMode = false;
//            } else if (gamepad1.dpad_left) {
//                holdSlidesMode = false;
//            } else if (gamepad1.left_bumper) {
//                holdSlidesMode = true;
//            }
//
//            if (holdSlidesMode && !gamepad1.dpad_up && !gamepad1.dpad_down) {
//                linearLeft.setPower(-0.1);
//                linearRight.setPower(0.1);
//            } else if (!gamepad1.dpad_up && !gamepad1.dpad_down) {
//                linearLeft.setPower(0);
//                linearRight.setPower(0);
//            }

            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            if (isModeTwo) {
                drive = -drive;
                strafe = -strafe;
            }

            double leftFrontPower = drive + strafe + turn;
            double rightFrontPower = -drive + strafe + turn;
            double leftBackPower = drive - strafe + turn;
            double rightBackPower = drive + strafe - turn;

            double max = Math.max(1.0, Math.max(Math.abs(leftFrontPower),
                    Math.max(Math.abs(rightFrontPower),
                            Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower)))));

            leftFront.setPower(leftFrontPower / max);
            rightFront.setPower(rightFrontPower / max);
            leftBack.setPower(leftBackPower / max);
            rightBack.setPower(rightBackPower / max);

            telemetry.addData("Mode", isModeTwo ? "Scoring Mode" : "Pickup Mode");;
            telemetry.addData("Hold Slides Mode", holdSlidesMode ? "Active" : "Inactive");
            telemetry.addData("Aiming Mode", isHoverMode ? "Aiming hover" : "Ground");
            telemetry.addData("Retract Mode", isRetractMode ? "Retract hover" : "Retracted");

            telemetry.addData("Motor Positions",
                    "LF: %d | LB: %d | RF: %d | RB: %d",
                    leftFront.getCurrentPosition(), leftBack.getCurrentPosition(),
                    rightFront.getCurrentPosition(), rightBack.getCurrentPosition());

//            telemetry.addData("Slide Motor Positions",
//                    "LL: %d | LR: %d",
//                    linearLeft.getCurrentPosition(), linearRight.getCurrentPosition());

            telemetry.addData("Motor Powers",
                    "LF: %.2f | LB: %.2f | RF: %.2f | RB: %.2f",
                    leftFront.getPower(), leftBack.getPower(), rightFront.getPower(), rightBack.getPower());

            telemetry.addData("Odometry", "X: %.2f | Y: %.2f | Hdg: %.2f",
                    odometry.getX(), odometry.getY(), odometry.getHeadingDegrees());
            telemetry.update();
        }
    }
}