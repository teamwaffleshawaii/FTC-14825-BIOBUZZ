package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Shared field-centric mecanum controls for TeleOp.
 *
 * Keeps the existing joystick signs, heading transform and D-pad behavior.
 * Road Runner autonomous trajectories continue to use MecanumDrive.
 */
public class DrivetrainSubsystem {
    private final Actuators actuator;
    private final Sensors sensor;
    private double resetAngle = 0;

    public DrivetrainSubsystem(Actuators actuator, Sensors sensor) {
        this.actuator = actuator;
        this.sensor = sensor;
    }

    public void drive(double leftStickX, double leftStickY, double rightStickX,
                      boolean robotCentric,
                      boolean dpadRight, boolean dpadLeft,
                      boolean dpadUp, boolean dpadDown) {
        double pRotate = -rightStickX;
        double stickX = -leftStickX * Math.sqrt(Math.pow(1 - Math.abs(pRotate), 2));
        double stickY = -leftStickY * Math.sqrt(Math.pow(1 - Math.abs(pRotate), 2));

        double gyroAngle = Math.toRadians(getHeadingDegrees());
        if (gyroAngle <= 0) {
            gyroAngle += Math.PI / 2;
        } else if (gyroAngle < Math.PI / 2) {
            gyroAngle += Math.PI / 2;
        } else {
            gyroAngle -= 3 * Math.PI / 2;
        }
        gyroAngle = -gyroAngle;

        if (robotCentric) {
            gyroAngle = -Math.PI / 2;
        }

        if (dpadRight) {
            stickX = 0.5;
        } else if (dpadLeft) {
            stickX = -0.5;
        }
        if (dpadUp) {
            stickY = -0.5;
        } else if (dpadDown) {
            stickY = 0.5;
        }

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

    public double getHeadingDegrees() {
        double heading = sensor.getHeading(AngleUnit.DEGREES);
        if (heading < -180) {
            heading += 360;
        } else if (heading > 180) {
            heading -= 360;
        }
        return heading - resetAngle;
    }

    public void stop() {
        actuator.leftFront(0);
        actuator.leftBack(0);
        actuator.rightFront(0);
        actuator.rightBack(0);
    }
}
