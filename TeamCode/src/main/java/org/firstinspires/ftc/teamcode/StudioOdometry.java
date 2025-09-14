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
    private static final double TICKS_PER_REV = 2000.0;   // REV Through-Bore Encoder (example)
    private static final double WHEEL_DIAMETER_IN = 1.26;  // Odometry wheel diameter (inches)
    private static final double TICKS_PER_INCH = TICKS_PER_REV / (Math.PI * WHEEL_DIAMETER_IN);
    private static final double TRACK_WIDTH = 8.5;       // Distance between left & right odo wheels (inches)

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

        // Three wheel odometry calculation with rotation correction
        double dTheta = (dR - dL) / TRACK_WIDTH; // radians
        double dxRobot, dyRobot;
        if (Math.abs(dTheta) < 1e-6) {
            dxRobot = -dS;
            dyRobot = (dL + dR) / 2.0;
        } else {
            double radiusForward = (dL + dR) / (2.0 * dTheta);
            double radiusStrafe = -dS / dTheta;
            dxRobot = radiusStrafe * Math.sin(dTheta) + radiusForward * (1 - Math.cos(dTheta));
            dyRobot = radiusForward * Math.sin(dTheta) - radiusStrafe * (1 - Math.cos(dTheta));
        }
        // Rotate robot-relative displacement into global coordinates
        double avgTheta = Math.toRadians(heading) + dTheta / 2.0;
        double cosT = Math.cos(avgTheta);
        double sinT = Math.sin(avgTheta);
        x += dxRobot * cosT - dyRobot * sinT;
        y += dxRobot * sinT + dyRobot * cosT;
        // Update heading in degrees and normalize
        heading += Math.toDegrees(dTheta);
        heading = (heading % 360 + 360) % 360;
    }

    // --- Getters ---
    public double getX() { return x; }              // inches
    public double getY() { return y; }              // inches
    public double getHeadingDegrees() { return heading; } // degrees [0, 360)
}