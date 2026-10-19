package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * STUDENT GUIDE: Alternate intake ON/OFF while transfer stays running.
 * This is a Road Runner Action that wraps a finite collectionAction.
 * It is available for experiments, but current AutoRight uses steady intake
 * instead. No Thread.sleep() is used, so motion can continue.
 * Non-blocking Road Runner action that pulses intake while transfer stays on.
 * Intended to run in parallel with a finite collection trajectory/action.
 */
public class PulsedIntakeAction implements Action {
    private final IntakeSubsystem intake;
    private final Actuators motors;
    private final Action collectionAction;
    private final double onSeconds;
    private final double offSeconds;
    private final ElapsedTime timer = new ElapsedTime();
    private boolean started = false;
    private boolean running = false;

    /** Require positive ON/OFF durations so the pulse cycle is meaningful. */
    public PulsedIntakeAction(IntakeSubsystem intake, Actuators motors, Action collectionAction, double onSeconds, double offSeconds) {
        if (onSeconds <= 0 || offSeconds <= 0) {
            throw new IllegalArgumentException("Pulse durations must be positive");
        }
        this.intake = intake;
        this.motors = motors;
        this.collectionAction = collectionAction;
        this.onSeconds = onSeconds;
        this.offSeconds = offSeconds;
    }

    @Override
    public boolean run(TelemetryPacket packet) {
        // The first run initializes motors and starts the timing cycle.
        if (!started) {
            started = true;
            running = true;
            timer.reset();
            motors.transferMotor.setPower(1);
            motors.intakeMotor.setPower(1);
        }

        // ElapsedTime lets us switch power without blocking drive updates.
        if (running && timer.seconds() >= onSeconds) {
            running = false;
            timer.reset();
            motors.intakeMotor.setPower(0);
        } else if (!running && timer.seconds() >= offSeconds) {
            running = true;
            timer.reset();
            motors.intakeMotor.setPower(1);
        }

        packet.put("Intake pulse", running ? "ON" : "OFF");
        // The wrapped collection path decides when this entire action ends.
        boolean active = collectionAction.run(packet);
        if (!active) stop();
        return active;
    }

    /** Stop intake and transfer when the collection path finishes. */
    public void stop() {
        intake.stopIntake();
    }
}
