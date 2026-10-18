package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

/** One-time shared robot initialization for TeleOp and Autonomous. */
public class Robot {
    public final Actuators actuator = new Actuators();
    public final Sensors sensor = new Sensors();
    public final IntakeSubsystem intake;
    public final DrivetrainSubsystem drivetrain;
    public final LauncherSubsystem launcher;

    public Robot(HardwareMap hardwareMap) {
        actuator.init(hardwareMap);
        sensor.init(hardwareMap);
        drivetrain = new DrivetrainSubsystem(actuator, sensor);
        intake = new IntakeSubsystem(actuator);
        launcher = new LauncherSubsystem(actuator, sensor, intake);
    }

    public void shutdown() {
        drivetrain.stop();
        launcher.cancel();
        intake.stopFeeding();
        sensor.stopLimelight();
    }
}
