package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * At a stationary autonomous shooting pose, sample alliance AprilTags before
 * feeding balls. Sets flywheel velocity and aims the turret using Limelight 3A.
 * After sampling, holds the locked velocity and turret position during feeding.
 */
public class AutoVisionLaunchAction implements Action {
    public static double SAMPLE_SECONDS = 0.5;
    public static double SETTLE_SECONDS = 0.5;

    private final Sensors sensors;
    private final LauncherSubsystem launcher;
    private final Actuators actuators;
    private final boolean redAlliance;
    private final AprilTagVelocityController estimator;
    private final ElapsedTime timer = new ElapsedTime();
    private boolean started = false;
    private boolean locked = false;
    private double selectedVelocity = LauncherSubsystem.TARGET_VELOCITY;
    private Double lockedTurretPosition = null;

    public AutoVisionLaunchAction(Sensors sensors, LauncherSubsystem launcher, Actuators actuators,
                                  boolean redAlliance) {
        this.sensors = sensors;
        this.launcher = launcher;
        this.actuators = actuators;
        this.redAlliance = redAlliance;
        this.estimator = new AprilTagVelocityController(redAlliance);
    }

    public AutoVisionLaunchAction(Sensors sensors, LauncherSubsystem launcher, boolean redAlliance) {
        this(sensors, launcher, null, redAlliance);
    }

    @Override
    public boolean run(TelemetryPacket packet) {
        if (!started) {
            timer.reset();
            started = true;
        }

        double sampleSeconds = Math.max(0, SAMPLE_SECONDS);
        if (!locked) {
            LLResult freshFrame = sensors.getFreshAprilTagResult();

            // 1. Velocity Control
            if (AprilTagVelocityController.ENABLED) {
                selectedVelocity = estimator.update(
                        freshFrame,
                        LauncherSubsystem.TARGET_VELOCITY, false);
            } else {
                selectedVelocity = LauncherSubsystem.TARGET_VELOCITY;
            }
            launcher.setFlywheelVelocity(selectedVelocity);

            // 2. Turret Auto-Aiming
            if (actuators != null) {
                Double turretTarget = TurretAimController.calculateTurretServoPosition(freshFrame, redAlliance);
                if (turretTarget != null) {
                    actuators.turretServo.setPosition(turretTarget);
                    lockedTurretPosition = turretTarget;
                }
            }

            if (timer.seconds() >= sampleSeconds) {
                locked = true;
                timer.reset();
            }
        }

        packet.put("Auto Vision Target (ticks/s)", selectedVelocity);
        packet.put("Auto Vision Tags", estimator.getCalibratedTagsSeen());
        packet.put("Auto Vision Locked", locked);
        packet.put("Auto Turret Target", lockedTurretPosition != null ? lockedTurretPosition : "Unchanged");
        packet.put("Auto Flywheel Actual (ticks/s)", launcher.getActualFlywheelVelocity());

        return !locked || timer.seconds() < Math.max(0, SETTLE_SECONDS);
    }
}
