package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Actuators;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Sensors;

@TeleOp (name = "TeleOp", group = "TeleOp")
public class TeleOperated extends LinearOpMode {

    Robot robot;
    Actuators actuator;
    Sensors sensor;
    IntakeSubsystem intakeSubsystem;
    DrivetrainSubsystem drivetrain;

    //Set the launch motor velocity
    double targetVelocity = 1060;
    //Read the actual velocity
    double currentVelocity = 0;
    //Check whether launch is ready
    boolean launcherReady = false;

    LauncherSubsystem launcher;

    @Override
    public void runOpMode() {

        robot = new Robot(hardwareMap);
        actuator = robot.actuator;
        sensor = robot.sensor;
        intakeSubsystem = robot.intake;
        drivetrain = robot.drivetrain;
        launcher = robot.launcher;

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                drive();
                resetAngle();
                intake();
                flowerRamp();
                launch();

                if (gamepad2.right_bumper) {
                    launcher.start();
                }
                launcher.update();

                launcherTelemetry();
                limelightTelemetry();
                telemetry.update();
            }
        }
        robot.shutdown();
    }

    public void drive() {
        drivetrain.drive(
                gamepad1.left_stick_x,
                gamepad1.left_stick_y,
                gamepad1.right_stick_x,
                gamepad1.right_bumper,
                gamepad1.dpad_right,
                gamepad1.dpad_left,
                gamepad1.dpad_up,
                gamepad1.dpad_down
        );
        telemetry.addData("Heading (Degrees)", drivetrain.getHeadingDegrees());
        telemetry.addData("Heading (Radians)", Math.toRadians(drivetrain.getHeadingDegrees()));
    }

    public void resetAngle() {
        if (gamepad1.start) {
            drivetrain.resetHeading();
        }
    }

    public void flowerRamp(){
        if (gamepad1.left_trigger_pressed || gamepad1.left_bumper){
            intakeSubsystem.rampDown();
        } else {
            intakeSubsystem.rampUp();
        }
    }
    public void intake() {
        // Automatic launcher owns the feed motors while active.
        if (launcher.isBusy()) return;

        if (gamepad1.a || gamepad2.a) {
            intakeSubsystem.intake(1);
        } else if (gamepad1.b || gamepad2.b) {
            intakeSubsystem.intake(-0.8);
        } else {
            intakeSubsystem.stopIntake();
        }
    }
    public void launch() {
        if (gamepad2.x) {
            actuator.launchMotor(targetVelocity); //adjust this as needed
            //later on, this should probably be auto-adjusted via Limelight data
        } else if (gamepad2.y) {
            actuator.launchMotor(0);
        }

        //Update actual velocity every loop
        currentVelocity = actuator.getLaunchVelocity();
        //Check whether launcher is ready
        launcherReady = actuator.isLaunchReady(targetVelocity);

        //Display launcher telemetry
        telemetry.addData("Target Velocity", targetVelocity);
        telemetry.addData("Actual Velocity", currentVelocity);
        telemetry.addData("Launcher Ready", launcherReady);

        if (launcherReady) {
            actuator.ledIndicator.setPosition(0.44); //green
        }
        else {
            actuator.ledIndicator.setPosition(0.28); //red
        }
    }
    public void launcherTelemetry() {
        telemetry.addData("Distance (cm)", launcher.getLastDistanceCm());
        telemetry.addData("Detecting Empty", launcher.isDetectingEmpty());
        telemetry.addData("Empty Timer", launcher.getEmptyDuration());
        telemetry.addData("Launch State", launcher.getState());
    }

    public void limelightTelemetry() {
        LLResult result = sensor.getLimelightResult();
        if (result != null && result.isValid()) {
            telemetry.addData("Limelight Target", "TX: %.2f°, TY: %.2f°, TA: %.2f%%",
                    result.getTx(), result.getTy(), result.getTa());
        } else {
            telemetry.addData("Limelight Target", "No Target");
        }
    }
}
