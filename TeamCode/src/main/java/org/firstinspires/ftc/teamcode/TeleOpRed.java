package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.TurretAimController;

/**
 * RED alliance TeleOp with automatic AprilTag turret aiming; BLUE extends this implementation.
 * Driving, intake, launcher velocity and ball feeding remain unchanged.
 * Automatic aiming starts enabled; disable AUTO_AIM_ENABLED for telemetry-only testing.
 */
@Config
@TeleOp(name = "TeleOp RED", group = "TeleOp")
public class TeleOpRed extends TeleOperated {
    // Automatic turret aiming is enabled by default.
    public static boolean AUTO_AIM_ENABLED = true;

    @Override
    protected boolean isRedAlliance() { return true; }

    @Override
    public void limelightTelemetry() {
        super.limelightTelemetry(); // Keep original TeleOp RED telemetry.

        LLResult frame = getTurretAprilTagFrame();
        double angleDegrees = TurretAimController.getHorizontalAngleDegrees(frame, isRedAlliance());
        double tangentialVelMps = drivetrain.getTangentialVelocityMps(angleDegrees);
        Double targetServo = TurretAimController.calculateTurretServoPosition(frame, isRedAlliance(), tangentialVelMps);

        telemetry.addData("Turret Mode", AUTO_AIM_ENABLED ? "AUTO AIM" : "TELEMETRY ONLY");

        if (targetServo != null) {
            telemetry.addData("Turret Suggested Servo", targetServo);
            telemetry.addData("Turret Actual Servo", actuator.turretServo.getPosition());
            telemetry.addData("Turret Cross Lead (m/s)", String.format(java.util.Locale.US, "%.2f", tangentialVelMps));

            if (AUTO_AIM_ENABLED && !launcher.isBusy()) {
                actuator.turretServo.setPosition(targetServo);
            }
        } else {
            telemetry.addData("Turret Aim", "No fresh alliance tags; holding servo");
        }
    }
}
