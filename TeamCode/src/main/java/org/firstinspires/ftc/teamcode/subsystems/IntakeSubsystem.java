package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

/** Shared controls for intake, transfer, step-ups and intake ramp. */
/**
 * STUDENT GUIDE: Higher-level controls for intake and ball transfer.
 * Several motors must cooperate to move flowers toward the launcher.
 * Keeping these commands together avoids repeating them in every OpMode.
 */
@Config
public class IntakeSubsystem {
    // Dashboard-adjustable pulse after the last A button is released.
    public static double STEPUP_RELEASE_REVERSE_SECONDS = 0.12;
    public static double STEPUP_RELEASE_REVERSE_POWER = 0.4;
    public static double TRANSFER_RELEASE_REVERSE_POWER = 0.2;
    private final ElapsedTime stepUpReverseTimer = new ElapsedTime();
    private boolean wasIntaking = false;
    private boolean stepUpReverseActive = false;
    private final Actuators actuator;

    public IntakeSubsystem(Actuators actuator) {
        this.actuator = actuator;
    }

    /** Run intake and transfer together; negative power reverses both. */
    public void intake(double power) {
        actuator.intakeMotor.setPower(power);
        actuator.transferMotor.setPower(power);
    }

    /** Stop intake and transfer; equivalent to calling intake(0). */
    public void stopIntake() {
        intake(0);
    }

    /** Update manual intake once per TeleOp loop; never block driving. */
    public void updateManualIntake(boolean intakeHeld, boolean reverseHeld, boolean launcherBusy) {
        if (wasIntaking && !intakeHeld && !reverseHeld && !launcherBusy) {
            startStepUpReversePulse();
        }
        wasIntaking = intakeHeld;

        // Launcher has exclusive control over feeding while busy.
        if (launcherBusy) {
            cancelStepUpReversePulse();
            return;
        }

        if (intakeHeld) {
            cancelStepUpReversePulse();
            intake(1);
            actuator.stepUpOff();
        } else if (reverseHeld) {
            cancelStepUpReversePulse();
            intake(-0.8);
            actuator.stepUpLeft.setPower(-1);
            actuator.stepUpRight.setPower(-1);
        } else {
            stopIntake();
            update();
        }
    }

    /** Start a timed reverse of the step-ups and transfer motor. */
    public void startStepUpReversePulse() {
        stepUpReverseTimer.reset();
        stepUpReverseActive = true;
    }

    /** Stop a pending pulse; the current controller sets servo power next. */
    public void cancelStepUpReversePulse() {
        stepUpReverseActive = false;
    }

    /** Advance the timed pulse while manual controls are idle. */
    public void update() {
        if (stepUpReverseActive
                && stepUpReverseTimer.seconds() < Math.max(0, STEPUP_RELEASE_REVERSE_SECONDS)) {
            double power = Math.max(0, Math.min(1, STEPUP_RELEASE_REVERSE_POWER));
            actuator.stepUpLeft.setPower(-power);
            actuator.stepUpRight.setPower(-power);
            // Intake stays stopped; transfer gently backs the leading ball down.
            double transferPower = Math.max(0, Math.min(1, TRANSFER_RELEASE_REVERSE_POWER));
            actuator.transferMotor.setPower(-transferPower);
        } else {
            stepUpReverseActive = false;
            actuator.stepUpOff();
        }
    }

    public boolean isStepUpReverseActive() {
        return stepUpReverseActive;
    }

    /** During launching, run step-ups and feed motors at their tuned powers. */
    public void feedLauncher() {
        cancelStepUpReversePulse();
        actuator.stepUpLeft.setPower(1);
        actuator.stepUpRight.setPower(1);
        actuator.intakeMotor.setPower(0.5);
        actuator.transferMotor.setPower(1);
    }

    /** Stop all feeding motors, including the step-up continuous servos. */
    public void stopFeeding() {
        actuator.stepUpLeft.setPower(0);
        actuator.stepUpRight.setPower(0);
        stopIntake();
    }

    /** Lower the intake ramp so flowers can enter. */
    public void rampDown() {
        actuator.intakeRampDown();
    }

    /** Raise the intake ramp after collection. */
    public void rampUp() {
        actuator.intakeRampUp();
    }
}
