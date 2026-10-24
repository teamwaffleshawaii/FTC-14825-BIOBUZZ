package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Road Runner Action to lock the turret onto AprilTags using Limelight 3A.
 * Can be executed sequentially or in parallel with drive trajectories.
 */
public class TurretAimAction implements Action {

    private final Sensors sensors;
    private final Actuators actuators;
    private final boolean redAlliance;
    private final double durationSeconds;
    private final ElapsedTime timer = new ElapsedTime();
    private boolean started = false;
    private Double lastLockedPosition = null;

    public TurretAimAction(Sensors sensors, Actuators actuators, boolean redAlliance, double durationSeconds) {
        this.sensors = sensors;
        this.actuators = actuators;
        this.redAlliance = redAlliance;
        this.durationSeconds = durationSeconds;
    }

    public TurretAimAction(Sensors sensors, Actuators actuators, boolean redAlliance) {
        this(sensors, actuators, redAlliance, 0.5);
    }

    @Override
    public boolean run(TelemetryPacket packet) {
        if (!started) {
            timer.reset();
            started = true;
        }

        LLResult frame = sensors.getFreshAprilTagResult();
        Double targetServo = TurretAimController.calculateTurretServoPosition(frame, redAlliance);

        if (targetServo != null) {
            actuators.turretServo.setPosition(targetServo);
            lastLockedPosition = targetServo;
            packet.put("Turret Aiming Servo", targetServo);
        } else if (lastLockedPosition != null) {
            packet.put("Turret Aiming Servo", "Locked at " + lastLockedPosition);
        } else {
            packet.put("Turret Aiming Servo", "Holding Center (0.46)");
        }

        return timer.seconds() < Math.max(0, durationSeconds);
    }
}
