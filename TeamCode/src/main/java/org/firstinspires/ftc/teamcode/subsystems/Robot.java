package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * STUDENT GUIDE: One shared place to initialize the whole robot.
 * Construct hardware first, then create subsystems that depend on it.
 * Both TeleOp and Autonomous can use the same subsystems and settings.
 * Calling shutdown() provides a common cleanup path when an OpMode ends.
 */
public class Robot {
    public final Actuators actuator = new Actuators();
    public final Sensors sensor = new Sensors();
    public final IntakeSubsystem intake;
    public final DrivetrainSubsystem drivetrain;
    public final LauncherSubsystem launcher;

    /** Initialize in dependency order: motors/sensors before their users. */
    public Robot(HardwareMap hardwareMap) {
        actuator.init(hardwareMap);
        sensor.init(hardwareMap);
        drivetrain = new DrivetrainSubsystem(actuator, sensor);
        intake = new IntakeSubsystem(actuator);
        launcher = new LauncherSubsystem(actuator, sensor, intake);
    }

    /**
     * Stop all driven motors and Limelight polling on exit.
     * Explicitly command zero flywheel velocity rather than relying only on
     * the FTC runtime's automatic motor stop at the end of an OpMode.
     */
    public void shutdown() {
        actuator.launchMotor(0);
        drivetrain.stop();
        launcher.cancel();
        intake.stopFeeding();
        sensor.stopLimelight();
    }
}
