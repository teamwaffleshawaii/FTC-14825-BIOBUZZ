package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Non-blocking Road Runner action that alternates intake and transfer motors.
 * Intended to run in parallel with a finite collection trajectory/action.
 */
public class PulsedIntakeAction implements Action {
    private final IntakeSubsystem intake;
    private final Action collectionAction;
    private final double onSeconds;
    private final double offSeconds;
    private final ElapsedTime timer = new ElapsedTime();
    private boolean started = false;
    private boolean running = false;

    public PulsedIntakeAction(IntakeSubsystem intake, Action collectionAction, double onSeconds, double offSeconds) {
        if (onSeconds <= 0 || offSeconds <= 0) {
            throw new IllegalArgumentException("Pulse durations must be positive");
        }
        this.intake = intake;
        this.collectionAction = collectionAction;
        this.onSeconds = onSeconds;
        this.offSeconds = offSeconds;
    }

    @Override
    public boolean run(TelemetryPacket packet) {
        if (!started) {
            started = true;
            running = true;
            timer.reset();
            intake.intake(1);
        }

        if (running && timer.seconds() >= onSeconds) {
            running = false;
            timer.reset();
            intake.stopIntake();
        } else if (!running && timer.seconds() >= offSeconds) {
            running = true;
            timer.reset();
            intake.intake(1);
        }

        packet.put("Intake pulse", running ? "ON" : "OFF");
        boolean active = collectionAction.run(packet);
        if (!active) stop();
        return active;
    }

    public void stop() {
        intake.stopIntake();
    }
}
