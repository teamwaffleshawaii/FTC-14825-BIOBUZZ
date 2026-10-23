package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * At a stationary autonomous shooting pose, sample alliance AprilTags before
 * feeding balls. Uses the SAME calibrated velocity estimator as TeleOp.
 * After sampling, hold the final target and allow time for the motor to settle.
 * If no tags are seen, the estimator uses the shared 1020 ticks/s baseline.
 *
 * This action only sets flywheel speed. The existing LauncherAction feeds balls.
 */
public class AutoVisionLaunchAction implements Action {
    // Tune these durations on the field. This action adds at most 1 second
    // at each shooting position, rather than waiting indefinitely for vision.
    public static double SAMPLE_SECONDS = 0.5;
    public static double SETTLE_SECONDS = 0.5;

    private final Sensors sensors;
    private final LauncherSubsystem launcher;
    private final AprilTagVelocityController estimator;
    private final ElapsedTime timer = new ElapsedTime();
    private boolean started = false;
    private boolean locked = false;
    private double selectedVelocity = LauncherSubsystem.TARGET_VELOCITY;

    public AutoVisionLaunchAction(Sensors sensors, LauncherSubsystem launcher,
                                  boolean redAlliance) {
        this.sensors = sensors;
        this.launcher = launcher;
        this.estimator = new AprilTagVelocityController(redAlliance);
    }

    @Override
    public boolean run(TelemetryPacket packet) {
        if (!started) {
            timer.reset();
            started = true;
        }

        double sampleSeconds = Math.max(0, SAMPLE_SECONDS);
        if (!locked) {
            if (AprilTagVelocityController.ENABLED) {
                selectedVelocity = estimator.update(
                        sensors.getFreshAprilTagResult(),
                        LauncherSubsystem.TARGET_VELOCITY, false);
            } else {
                selectedVelocity = LauncherSubsystem.TARGET_VELOCITY;
            }
            launcher.setFlywheelVelocity(selectedVelocity);
            if (timer.seconds() >= sampleSeconds) {
                // Lock velocity for the upcoming volley. Do not chase new
                // camera readings once the sampling window is finished.
                locked = true;
                timer.reset();
            }
        }

        packet.put("Auto Vision Target (ticks/s)", selectedVelocity);
        packet.put("Auto Vision Tags", estimator.getCalibratedTagsSeen());
        packet.put("Auto Vision Locked", locked);
        packet.put("Auto Flywheel Actual (ticks/s)",
                launcher.getActualFlywheelVelocity());

        return !locked || timer.seconds() < Math.max(0, SETTLE_SECONDS);
    }
}
