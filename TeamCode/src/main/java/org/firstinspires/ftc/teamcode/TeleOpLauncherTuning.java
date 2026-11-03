package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;

/**
 * Competition controls with a manually selected flywheel velocity for tuning.
 * Inherits the competition driving, intake, ramp, ball counter, flywheel X/Y,
 * right-bumper volley, and Y emergency cancel behavior.
 * Unlike competition TeleOp, AprilTags cannot change the target velocity.
 */
@Config
@TeleOp(name = "TeleOp-LauncherTuning", group = "Tuning")
public class TeleOpLauncherTuning extends TeleOperated {
    public static double TARGET_TICKS_PER_SECOND = 1020.0;

    @Override
    public void launch() {
        // Reuse competition launch controls and telemetry, but replace the
        // shared fallback target with the Dashboard-controlled tuning target.
        // The superclass uses the AprilTag controller only when ENABLED is true.
        // Avoid modifying that global flag: other OpModes must remain unchanged.
        double originalBaseline = LauncherSubsystem.TARGET_VELOCITY;
        boolean originalVisionEnabled =
                org.firstinspires.ftc.teamcode.subsystems.AprilTagVelocityController.ENABLED;
        try {
            LauncherSubsystem.TARGET_VELOCITY = Math.max(0, TARGET_TICKS_PER_SECOND);
            org.firstinspires.ftc.teamcode.subsystems.AprilTagVelocityController.ENABLED = false;
            super.launch();
        } finally {
            LauncherSubsystem.TARGET_VELOCITY = originalBaseline;
            org.firstinspires.ftc.teamcode.subsystems.AprilTagVelocityController.ENABLED =
                    originalVisionEnabled;
        }
    }
}
