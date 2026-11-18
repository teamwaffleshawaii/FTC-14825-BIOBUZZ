package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Encoder-position PIDF for the defender linear slide.
 * Start each OpMode with the slide physically at its retracted zero.
 * Never use the encoder reset while the slide is extended.
 */
@Config
public class DefenderSubsystem {
    public static int RETRACTED_TICKS = 0;
    public static int EXTENDED_TICKS = 1100;
    public static double P = 0.015;
    public static double I = 0.0;
    public static double D = 0.0;
    public static double F = 0.001;
    public static double MAX_POWER = 1.00;
    public static double POSITION_TOLERANCE = 12.0;
    public static double INTEGRAL_LIMIT = 1000.0;

    private final DcMotorEx motor;
    private final ElapsedTime timer = new ElapsedTime();
    private double target = 0;
    private double integral = 0;
    private double lastError = 0;
    private double commandedPower = 0;
    private boolean firstUpdate = true;

    public DefenderSubsystem(Actuators actuators) {
        motor = actuators.defenderMotor;
        // This establishes zero ONLY if the slide is physically retracted at init.
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        timer.reset();
    }

    public void update(boolean extendHeld) {
        int extension = Math.max(0, EXTENDED_TICKS);
        target = extendHeld ? extension : Math.max(0, Math.min(extension, RETRACTED_TICKS));
        double position = motor.getCurrentPosition();
        double error = target - position;
        double dt = timer.seconds();
        timer.reset();
        if (firstUpdate || dt <= 0 || dt > 0.25) {
            dt = 0;
            firstUpdate = false;
        }
        if (Math.abs(error) <= Math.max(0, POSITION_TOLERANCE)) {
            integral = 0;
            commandedPower = 0;
        } else {
            if (dt > 0) {
                integral += error * dt;
                double limit = Math.max(0, INTEGRAL_LIMIT);
                integral = Math.max(-limit, Math.min(limit, integral));
            }
            double derivative = dt > 0 ? (error - lastError) / dt : 0;
            double feedforward = F * Math.signum(error);
            double output = P * error + I * integral + D * derivative + feedforward;
            double maxPower = Math.max(0, Math.min(1, MAX_POWER));
            commandedPower = Math.max(-maxPower, Math.min(maxPower, output));
        }
        lastError = error;
        // Never drive farther past the commanded software travel endpoints.
        if ((position <= 0 && commandedPower < 0) ||
                (position >= extension && commandedPower > 0)) {
            commandedPower = 0;
        }
        motor.setPower(commandedPower);
    }

    public int getPosition() { return motor.getCurrentPosition(); }
    public double getTarget() { return target; }
    public double getError() { return target - getPosition(); }
    public double getPower() { return commandedPower; }

    public void stop() {
        motor.setPower(0);
        commandedPower = 0;
    }
}
