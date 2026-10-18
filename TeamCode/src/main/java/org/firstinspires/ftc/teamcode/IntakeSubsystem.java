package org.firstinspires.ftc.teamcode;

/** Shared controls for intake, transfer, step-ups and intake ramp. */
public class IntakeSubsystem {
    private final Actuators actuator;

    public IntakeSubsystem(Actuators actuator) {
        this.actuator = actuator;
    }

    public void intake(double power) {
        actuator.intakeMotor.setPower(power);
        actuator.transferMotor.setPower(power);
    }

    public void stopIntake() {
        intake(0);
    }

    public void feedLauncher() {
        actuator.stepUpLeft.setPower(1);
        actuator.stepUpRight.setPower(1);
        actuator.intakeMotor.setPower(0.5);
        actuator.transferMotor.setPower(1);
    }

    public void stopFeeding() {
        actuator.stepUpLeft.setPower(0);
        actuator.stepUpRight.setPower(0);
        stopIntake();
    }

    public void rampDown() {
        actuator.intakeRampDown();
    }

    public void rampUp() {
        actuator.intakeRampUp();
    }
}
