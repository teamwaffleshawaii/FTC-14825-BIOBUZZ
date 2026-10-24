package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

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

    public DrivetrainSubsystem(Actuators actuator, Sensors sensor) {
        this.actuator = actuator;
        this.sensor = sensor;
    }

    /**
     * Convert joystick commands into four mecanum wheel powers.
     * Field-centric driving rotates joystick directions using robot heading.
     * The right bumper switches to robot-centric control; D-pad overrides
     * translation for small controlled movements.
     */
    public void drive(double leftStickX, double leftStickY, double rightStickX,
                      boolean robotCentric,
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

        // Robot-centric means forward follows the robot's front, not the field.
        if (robotCentric) {
            gyroAngle = -Math.PI / 2;
        }

        // D-pad signs verified against driver-reported physical motion:
        // Up = forward, Down = backward, Left = left, Right = right.
        if (dpadRight) {
            stickX = -0.5;
        } else if (dpadLeft) {
            stickX = 0.5;
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
