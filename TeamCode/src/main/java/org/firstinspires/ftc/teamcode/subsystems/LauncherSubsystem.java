package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * STUDENT GUIDE: Controls the four-ball launch sequence in TeleOp and Auto.
 *
 * This is a NON-BLOCKING state machine: instead of sleeping for a long time,
 * update() checks timers each loop so the robot can keep doing other work.
 * The launcher uses encoder feedback and PIDF to maintain flywheel speed.
 *
 * Future plan: select a velocity using a measured AprilTag-to-goal distance.
 * That feature is NOT implemented or enabled yet.
 */
@Config
public class LauncherSubsystem {
    // PIDF is the motor controller's velocity feedback tuning:
    // P = respond to current speed error; I = accumulated error;
    // D = change in error; F = feedforward estimate of motor effort.
    // These are the team's current tested values, not universal constants.
    // @Config exposes them in FTC Dashboard while the OpMode runs.
    // Dashboard edits are temporary until we save new defaults in Java.
    // Changes are applied to hardware only when a coefficient changes.
    public static double LAUNCH_P = 65.0;
    public static double LAUNCH_I = 0.0;
    public static double LAUNCH_D = 0.0;
    public static double LAUNCH_F = 20;

    // TeleOp shooting speed, in encoder ticks/second. Change live in Dashboard.
    // Autonomous keeps its existing independent targetVelocity setting.
    public static double TELEOP_TARGET_VELOCITY = 1060.0;

    // Future vision-based velocity control is deliberately disabled until calibrated.
    // This flag is a PLACEHOLDER, not a working automatic aiming switch.
    // Even setting it true today will NOT change launcher velocity.
    public static boolean AUTO_VELOCITY_ENABLED = false;

    // Remember the values last sent to the motor controller.
    // NaN (not-a-number) guarantees the first applyPIDF() writes all values.
    private double appliedP = Double.NaN;
    private double appliedI = Double.NaN;
    private double appliedD = Double.NaN;
    private double appliedF = Double.NaN;
    // Each state represents one stage of the four-ball feeding sequence.
    // IDLE: waiting; FEEDING: moving balls; KICKER_UP: activating kicker;
    // RESETTING: lowering kicker before returning to IDLE.
    public enum State { IDLE, FEEDING, KICKER_UP, RESETTING }

    private final Actuators actuator;
    private final Sensors sensor;
    private final IntakeSubsystem intake;
    private final ElapsedTime distanceTimer = new ElapsedTime();
    private final ElapsedTime kickerTimer = new ElapsedTime();

    // Timing values are in SECONDS. Tune cautiously on the actual robot.
    public double emptyDelay = 0.02; // TeleOp: 20 ms
    public double emptyDelayAutoOnly = 0.04; // Autonomous: 40 ms
    public double kickerUpDelay = 0.7;
    public double kickerResetDelay = 0.4;
    // Flywheel target uses encoder ticks per second, NOT RPM.
    // TeleOp currently also has its own 1060 target; we'll consolidate later.
    public double targetVelocity = 1060;

    private State state = State.IDLE;
    private boolean detectingEmpty = false;
    private double activeEmptyDelay = emptyDelay;
    private boolean completed = false;
    private double lastDistanceCm = Double.NaN;

    public LauncherSubsystem(Actuators actuator, Sensors sensor, IntakeSubsystem intake) {
        this.actuator = actuator;
        this.sensor = sensor;
        this.intake = intake;
        applyPIDF();
    }

    /** Apply Dashboard edits without rewriting unchanged coefficients every loop. */
    public void applyPIDF() {
        if (LAUNCH_P != appliedP || LAUNCH_I != appliedI ||
                LAUNCH_D != appliedD || LAUNCH_F != appliedF) {
            actuator.launchMotor.setVelocityPIDFCoefficients(
                    LAUNCH_P, LAUNCH_I, LAUNCH_D, LAUNCH_F);
            appliedP = LAUNCH_P;
            appliedI = LAUNCH_I;
            appliedD = LAUNCH_D;
            appliedF = LAUNCH_F;
        }
    }

    /** Begin a feeding sequence; ignore duplicate starts while busy. */
    public void start() {
        startWithEmptyDelay(emptyDelay);
    }

    /** Autonomous uses the same feed sequence with a longer empty confirmation. */
    public void startAuto() {
        startWithEmptyDelay(emptyDelayAutoOnly);
    }

    private void startWithEmptyDelay(double delaySeconds) {
        if (isBusy()) return;
        activeEmptyDelay = Math.max(0, delaySeconds);
        completed = false;
        detectingEmpty = false;
        distanceTimer.reset();
        intake.feedLauncher();
        state = State.FEEDING;
    }

    /**
     * Call frequently from the OpMode loop. Also checks FTC Dashboard PIDF
     * edits when no ball-feeding sequence is active.
     */
    public void update() {
        applyPIDF();
        if (!isBusy()) return;

        // Continue commanding the feed motors until the sequence finishes.
        intake.feedLauncher();
        switch (state) {
            case FEEDING:
                // The distance sensor checks whether balls are still present.
                // Require a brief continuous 'empty' reading before kicking.
                lastDistanceCm = sensor.getDistance(DistanceUnit.CM);
                if (Double.isFinite(lastDistanceCm) && lastDistanceCm > 10.0) {
                    if (!detectingEmpty) {
                        detectingEmpty = true;
                        distanceTimer.reset();
                    }
                    if (distanceTimer.seconds() >= activeEmptyDelay) {
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
                // Hold the kicker up for the configured interval.
                if (kickerTimer.seconds() >= kickerUpDelay) {
                    actuator.kickerDown();
                    kickerTimer.reset();
                    state = State.RESETTING;
                }
                break;
            case RESETTING:
                // Return the kicker to its starting position, then stop feed.
                if (kickerTimer.seconds() >= kickerResetDelay) {
                    intake.stopFeeding();
                    detectingEmpty = false;
                    completed = true;
                    state = State.IDLE;
                }
                break;
            default:
                break;
        }
    }

    /** Safely stop the feeding sequence (does not turn off the flywheel). */
    public void cancel() {
        intake.stopFeeding();
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
    /** Request a motor speed in encoder ticks/second using the current PIDF. */
    public void setFlywheelVelocity(double velocity) {
        applyPIDF();
        actuator.launchMotor(velocity);
    }
    /** True when actual flywheel speed is within the hardware readiness band. */
    public boolean isFlywheelReady() {
        return actuator.isLaunchReady(targetVelocity);
    }
}
