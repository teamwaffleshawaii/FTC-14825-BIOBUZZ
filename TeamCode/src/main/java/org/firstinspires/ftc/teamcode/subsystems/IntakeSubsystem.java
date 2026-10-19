package org.firstinspires.ftc.teamcode.subsystems;

/** Shared controls for intake, transfer, step-ups and intake ramp. */
/**
 * STUDENT GUIDE: Higher-level controls for intake and ball transfer.
 * Several motors must cooperate to move flowers toward the launcher.
 * Keeping these commands together avoids repeating them in every OpMode.
 */
public class IntakeSubsystem {
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

    /** During launching, run step-ups and feed motors at their tuned powers. */
    public void feedLauncher() {
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
