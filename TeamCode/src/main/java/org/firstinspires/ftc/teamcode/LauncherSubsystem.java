package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Shared, non-blocking four-ball launch sequence for TeleOp and Autonomous.
 * Call update() every loop while a sequence is running.
 */
public class LauncherSubsystem {
    public enum State { IDLE, FEEDING, KICKER_UP, RESETTING }

    private final Actuators actuator;
    private final Sensors sensor;
    private final ElapsedTime distanceTimer = new ElapsedTime();
    private final ElapsedTime kickerTimer = new ElapsedTime();

    public double emptyDelay = 0.05;
    public double kickerUpDelay = 0.4;
    public double kickerResetDelay = 0.4;
    public double targetVelocity = 1060;

    private State state = State.IDLE;
    private boolean detectingEmpty = false;
    private boolean completed = false;
    private double lastDistanceCm = Double.NaN;

    public LauncherSubsystem(Actuators actuator, Sensors sensor) {
        this.actuator = actuator;
        this.sensor = sensor;
    }

    public void start() {
        if (isBusy()) return;
        completed = false;
        detectingEmpty = false;
        distanceTimer.reset();
        setFeedPower(true);
        state = State.FEEDING;
    }

    public void update() {
        if (!isBusy()) return;

        // Continue commanding the feed motors until the sequence finishes.
        setFeedPower(true);
        switch (state) {
            case FEEDING:
                lastDistanceCm = sensor.getDistance(DistanceUnit.CM);
                if (Double.isFinite(lastDistanceCm) && lastDistanceCm > 5.0) {
                    if (!detectingEmpty) {
                        detectingEmpty = true;
                        distanceTimer.reset();
                    }
                    if (distanceTimer.seconds() >= emptyDelay) {
                        actuator.kickerUp();
                        kickerTimer.reset();
                        state = State.KICKER_UP;
                    }
                } else {
                    detectingEmpty = false;
                    distanceTimer.reset();
                }
                break;
            case KICKER_UP:
                if (kickerTimer.seconds() >= kickerUpDelay) {
                    actuator.kickerDown();
                    kickerTimer.reset();
                    state = State.RESETTING;
                }
                break;
            case RESETTING:
                if (kickerTimer.seconds() >= kickerResetDelay) {
                    setFeedPower(false);
                    detectingEmpty = false;
                    completed = true;
                    state = State.IDLE;
                }
                break;
            default:
                break;
        }
    }

    private void setFeedPower(boolean enabled) {
        actuator.stepUpLeft.setPower(enabled ? 1 : 0);
        actuator.stepUpRight.setPower(enabled ? 1 : 0);
        actuator.intakeMotor.setPower(enabled ? 0.5 : 0);
        actuator.transferMotor.setPower(enabled ? 1 : 0);
    }

    public void cancel() {
        setFeedPower(false);
        actuator.kickerDown();
        detectingEmpty = false;
        completed = false;
        state = State.IDLE;
    }

    public boolean isBusy() { return state != State.IDLE; }
    public boolean isComplete() { return completed; }
    public State getState() { return state; }
    public double getLastDistanceCm() { return lastDistanceCm; }
    public boolean isDetectingEmpty() { return detectingEmpty; }
    public double getEmptyDuration() {
        return detectingEmpty ? distanceTimer.seconds() : 0.0;
    }
    public void setFlywheelVelocity(double velocity) {
        actuator.launchMotor(velocity);
    }
    public boolean isFlywheelReady() {
        return actuator.isLaunchReady(targetVelocity);
    }
}
