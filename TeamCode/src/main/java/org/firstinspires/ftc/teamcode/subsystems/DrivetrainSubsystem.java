package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Shared field-centric mecanum controls for TeleOp.
 *
 * Keeps the existing joystick signs and heading transform.
 * D-pad translation signs are corrected for the physical robot.
 * Road Runner autonomous trajectories continue to use MecanumDrive.
 */
public class DrivetrainSubsystem {
    // Actuators sends motor power; Sensors supplies heading information.
    // resetAngle is the driver's chosen field-heading offset.
    private final Actuators actuator;
    private final Sensors sensor;
    private double resetAngle = 0;

    // goBILDA Yellow Jacket 435 RPM (13.7:1), direct drive to 104 mm wheels.
    // Encoder resolution is 384.5 pulses per output-shaft revolution.
    private static final double DRIVE_ENCODER_TICKS_PER_REV = 384.5;
    private static final double DRIVE_WHEEL_DIAMETER_METERS = 0.104;
    private static final double DRIVE_METERS_PER_TICK =
            Math.PI * DRIVE_WHEEL_DIAMETER_METERS / DRIVE_ENCODER_TICKS_PER_REV;

    // Full commanded power for D-pad left/right strafing.
    private static final double DPAD_STRAFE_POWER = 0.85;

    private final ElapsedTime driveVelocityTimer = new ElapsedTime();
    private boolean driveVelocityInitialized = false;
    private int previousLeftFrontTicks;
    private int previousRightFrontTicks;
    private int previousLeftBackTicks;
    private int previousRightBackTicks;
    private double robotForwardVelocityMps = 0.0;
    private double robotStrafeVelocityMps = 0.0;

    public DrivetrainSubsystem(Actuators actuator, Sensors sensor) {
        this.actuator = actuator;
        this.sensor = sensor;
    }

    /**
     * Convert joystick commands into four mecanum wheel powers.
     * Field-centric driving rotates joystick directions using robot heading.
     * Field-centric translation is always used; D-pad overrides
     * translation for small controlled movements.
     */
    public void drive(double leftStickX, double leftStickY, double rightStickX,
                      boolean dpadRight, boolean dpadLeft,
                      boolean dpadUp, boolean dpadDown) {
        // Right stick controls turning. Reduce translation when rotating.
        double pRotate = -rightStickX;
        double stickX = -leftStickX * Math.sqrt(Math.pow(1 - Math.abs(pRotate), 2));
        double stickY = -leftStickY * Math.sqrt(Math.pow(1 - Math.abs(pRotate), 2));

        // Convert degrees to radians for Math.sin/atan2 calculations.
        // Preserve this robot's existing heading-offset convention.
        double gyroAngle = Math.toRadians(getHeadingDegrees());
        if (gyroAngle <= 0) {
            gyroAngle += Math.PI / 2;
        } else if (gyroAngle < Math.PI / 2) {
            gyroAngle += Math.PI / 2;
        } else {
            gyroAngle -= 3 * Math.PI / 2;
        }
        gyroAngle = -gyroAngle;

        // D-pad signs verified against driver-reported physical motion:
        // Up = forward, Down = backward, Left = left, Right = right.
        if (dpadRight) {
            stickX = -DPAD_STRAFE_POWER;
        } else if (dpadLeft) {
            stickX = DPAD_STRAFE_POWER;
        }
        if (dpadUp) {
            stickY = 0.5;
        } else if (dpadDown) {
            stickY = -0.5;
        }

        // Rotate the desired translation vector and mix it with rotation.
        // Diagonal wheel combinations produce mecanum strafing.
        double theta = Math.atan2(stickY, stickX) - gyroAngle - Math.PI / 2;
        double pX = Math.hypot(stickX, stickY) * Math.sin(theta + Math.PI / 4);
        double pY = Math.hypot(stickX, stickY) * Math.sin(theta - Math.PI / 4);

        actuator.leftFront(pY - pRotate);
        actuator.leftBack(pX - pRotate);
        actuator.rightBack(pY + pRotate);
        actuator.rightFront(pX + pRotate);

        // Estimate chassis translation from encoder deltas without changing
        // the existing RUN_WITHOUT_ENCODER drivetrain control.
        updateDriveVelocityEstimate();
    }

    /**
     * Estimate robot-centric forward and rightward velocity from the four
     * mecanum wheel encoders. Opposing wheel contributions cancel rotation.
     */
    private void updateDriveVelocityEstimate() {
        int lf = actuator.leftFront.getCurrentPosition();
        int rf = actuator.rightFront.getCurrentPosition();
        int lb = actuator.leftBack.getCurrentPosition();
        int rb = actuator.rightBack.getCurrentPosition();
        double dt = driveVelocityTimer.seconds();

        if (!driveVelocityInitialized || dt <= 0.001 || dt > 0.25) {
            previousLeftFrontTicks = lf;
            previousRightFrontTicks = rf;
            previousLeftBackTicks = lb;
            previousRightBackTicks = rb;
            driveVelocityTimer.reset();
            driveVelocityInitialized = true;
            robotForwardVelocityMps = 0.0;
            robotStrafeVelocityMps = 0.0;
            return;
        }

        double lfMps = (lf - previousLeftFrontTicks) * DRIVE_METERS_PER_TICK / dt;
        double rfMps = (rf - previousRightFrontTicks) * DRIVE_METERS_PER_TICK / dt;
        double lbMps = (lb - previousLeftBackTicks) * DRIVE_METERS_PER_TICK / dt;
        double rbMps = (rb - previousRightBackTicks) * DRIVE_METERS_PER_TICK / dt;

        previousLeftFrontTicks = lf;
        previousRightFrontTicks = rf;
        previousLeftBackTicks = lb;
        previousRightBackTicks = rb;
        driveVelocityTimer.reset();

        // Inverse mecanum kinematics. Positive strafe is robot-right.
        double forward = (lfMps + rfMps + lbMps + rbMps) * 0.25;
        double strafe = (-lfMps + rfMps + lbMps - rbMps) * 0.25;

        // Light filtering reduces encoder quantization without adding much lag.
        robotForwardVelocityMps += 0.35 * (forward - robotForwardVelocityMps);
        robotStrafeVelocityMps += 0.35 * (strafe - robotStrafeVelocityMps);
    }

    public double getRobotForwardVelocityMps() {
        return robotForwardVelocityMps;
    }

    public double getRobotStrafeVelocityMps() {
        return robotStrafeVelocityMps;
    }

    /**
     * Positive result means robot translation is toward the observed goal.
     * Limelight TX is relative to the chassis because the camera is fixed.
     */
    public double getVelocityTowardTargetMps(double targetTxDegrees) {
        double angle = Math.toRadians(targetTxDegrees);
        return robotForwardVelocityMps * Math.cos(angle)
                + robotStrafeVelocityMps * Math.sin(angle);
    }

    /** Preserve the existing driver's heading-reset behavior. */
    public void resetHeading() {
        resetAngle += getHeadingDegrees();
    }

    /** Return driver-relative heading, including any manual reset offset. */
    public double getHeadingDegrees() {
        double heading = sensor.getHeading(AngleUnit.DEGREES);
        if (heading < -180) {
            heading += 360;
        } else if (heading > 180) {
            heading -= 360;
        }
        return heading - resetAngle;
    }

    /** Command all four drive motors to zero power. */
    public void stop() {
        actuator.leftFront(0);
        actuator.leftBack(0);
        actuator.rightFront(0);
        actuator.rightBack(0);
    }
}
