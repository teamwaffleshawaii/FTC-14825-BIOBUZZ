package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.TurretAimController;

/**
 * Experimental turret aiming for RED; BLUE extends this shared implementation.
 * Driving, intake, launcher velocity and ball feeding remain unchanged.
 * Starts with automatic aiming enabled; test with balls removed.
 */
@Config
@TeleOp(name = "TeleOpTurret-RED", group = "TeleOp")
public class TeleOpTurretRed extends TeleOperated {
    // Automatic aiming is enabled for this experimental OpMode.
    public static boolean AUTO_AIM_ENABLED = true;

    @Override
    protected boolean isRedAlliance() { return true; }

    @Override
    public void limelightTelemetry() {
        super.limelightTelemetry(); // Keep original TeleOp RED telemetry.

        LLResult frame = getTurretAprilTagFrame();
        Double targetServo = TurretAimController.calculateTurretServoPosition(frame, isRedAlliance());

        telemetry.addData("Turret Mode", AUTO_AIM_ENABLED ? "AUTO AIM" : "TELEMETRY ONLY");

        if (targetServo != null) {
            telemetry.addData("Turret Suggested Servo", targetServo);
            telemetry.addData("Turret Actual Servo", actuator.turretServo.getPosition());

            if (AUTO_AIM_ENABLED && !launcher.isBusy()) {
                actuator.turretServo.setPosition(targetServo);
            }
        } else {
            telemetry.addData("Turret Aim", "No fresh alliance tags; holding servo");
        }
    }
}
