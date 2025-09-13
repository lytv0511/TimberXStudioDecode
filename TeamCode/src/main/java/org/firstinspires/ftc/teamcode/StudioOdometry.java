package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class StudioOdometry {
    private DcMotor leftOdo, rightOdo, strafeOdo;
    // Robot pose (relative to start, in inches and degrees)
    private double x = 0.0;       // left/right (inches)
    private double y = 0.0;       // forward/back (inches)
    private double heading = 0.0; // degrees [0, 360)

    // Previous encoder values
    private int prevLeft = 0, prevRight = 0, prevStrafe = 0;

    // Tuning constants (measure on your robot!)
    private static final double TICKS_PER_REV = 8192.0;   // REV Through-Bore Encoder (example)
    private static final double WHEEL_DIAMETER_IN = 1.5;  // Odometry wheel diameter (inches)
    private static final double TICKS_PER_INCH = TICKS_PER_REV / (Math.PI * WHEEL_DIAMETER_IN);
    private static final double TRACK_WIDTH = 15.0;       // Distance between left & right odo wheels (inches)

    public StudioOdometry(HardwareMap hardwareMap) {
        leftOdo = hardwareMap.get(DcMotor.class, "leftOdo");
        rightOdo = hardwareMap.get(DcMotor.class, "rightOdo");
        strafeOdo = hardwareMap.get(DcMotor.class, "strafeOdo");
        reset();
    }

    /** Reset pose and align encoders */
    public void reset() {
        // Reset encoders to 0
        leftOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        strafeOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Put encoders back into run mode so they start counting again
        leftOdo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightOdo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        strafeOdo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Reset internal pose variables
        prevLeft = 0;
        prevRight = 0;
        prevStrafe = 0;
        x = 0.0;
        y = 0.0;
        heading = 0.0;
    }

    /** Update robot position and heading using encoder deltas */
    public void update() {
        // Current encoder ticks
        int leftTicks = leftOdo.getCurrentPosition();
        int rightTicks = -rightOdo.getCurrentPosition();
        int strafeTicks = strafeOdo.getCurrentPosition();

        // Delta ticks since last update
        int dLeft = leftTicks - prevLeft;
        int dRight = rightTicks - prevRight;
        int dStrafe = strafeTicks - prevStrafe;

        // Save for next cycle
        prevLeft = leftTicks;
        prevRight = rightTicks;
        prevStrafe = strafeTicks;

        // Convert ticks to inches
        double dL = dLeft / TICKS_PER_INCH;
        double dR = dRight / TICKS_PER_INCH;
        double dS = dStrafe / TICKS_PER_INCH;

        // Update pose in robot-relative coordinates
        double forward = -(dL + dR) / 2.0 * 3.9717563986 * 1.1303344867; // forward/back average
        double strafe = -dS * 3.9106145251 * 1.1303344867 * 0.8884940027 * 0.865248227;// left/right

        y += forward;
        x += strafe;

        // Heading change in degrees
        double dThetaDeg = Math.toDegrees((dR - dL) / TRACK_WIDTH);
        heading += dThetaDeg * 3.164;

        // Normalize heading to [0, 360)
        heading = (heading % 360 + 360) % 360;
    }

    // --- Getters ---
    public double getX() { return x; }              // inches
    public double getY() { return y; }              // inches
    public double getHeadingDegrees() { return heading; } // degrees [0, 360)
}