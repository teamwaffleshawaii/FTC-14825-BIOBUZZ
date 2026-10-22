package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Actuators;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Sensors;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * STUDENT GUIDE: Driver-controlled robot program (TeleOp).
 * Robot owns the shared hardware and subsystems; this class maps gamepad
 * inputs to commands and displays information on the Driver Station.
 * The while loop must run repeatedly so driving, timers and sensors update.
 */
@TeleOp (name = "TeleOp", group = "TeleOp")
public class TeleOperated extends LinearOpMode {

    // References to the SAME objects created by Robot; no extra hardware setup.
    Robot robot;
    Actuators actuator;
    Sensors sensor;
    IntakeSubsystem intakeSubsystem;
    DrivetrainSubsystem drivetrain;

    // TeleOp uses LauncherSubsystem.TELEOP_TARGET_VELOCITY so students can
    // adjust shooting speed live in FTC Dashboard without redeploying.
    // Remember whether X started the flywheel so changes apply while spinning.
    boolean flywheelEnabled = false;
    //Read the actual velocity
    double currentVelocity = 0;
    //Check whether launch is ready
    boolean launcherReady = false;

    LauncherSubsystem launcher;

    // ElapsedTime measures real elapsed time without pausing the robot loop.
    // It is used to blink the indicator LED without sleep().
    private final ElapsedTime ledTimer = new ElapsedTime();

    // FTC SDK calls runOpMode() when the driver selects and starts this OpMode.
    @Override
    public void runOpMode() {
        // Send the same telemetry to Driver Station and FTC Dashboard.
        // Dashboard can graph Target Velocity and Actual Velocity together
        // while students adjust launcher PIDF values live.
        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );


        robot = new Robot(hardwareMap);
        actuator = robot.actuator;
        sensor = robot.sensor;
        intakeSubsystem = robot.intake;
        drivetrain = robot.drivetrain;
        launcher = robot.launcher;

        // Initialization happens BEFORE the match; wait until PLAY is pressed.
        waitForStart();

        if (opModeIsActive()) {
            // Main loop: read controls, update subsystems, then send telemetry.
            // Do not insert long sleeps here; they delay every robot control.
            while (opModeIsActive()) {
                drive();
                resetAngle();
                intake();
                flowerRamp();
                launch();

                // Gamepad 2 right bumper begins the automatic four-ball feed.
                // launcher.start() ignores new starts while already busy.
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

    /** Translate driver joystick/D-pad inputs into mecanum motor commands. */
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

    /** Re-zero the driver's field-centric heading when START is pressed. */
    public void resetAngle() {
        if (gamepad1.start) {
            drivetrain.resetHeading();
        }
    }

    /** Move the intake ramp with gamepad 1 trigger or bumper. */
    public void flowerRamp(){
        if (gamepad1.left_trigger_pressed || gamepad1.left_bumper){
            intakeSubsystem.rampDown();
        } else {
            intakeSubsystem.rampUp();
        }
    }
    /** Map either driver's A/B buttons to the intake subsystem. */
    public void intake() {
        intakeSubsystem.updateManualIntake(
                gamepad1.a || gamepad2.a,
                gamepad1.b || gamepad2.b,
                launcher.isBusy()
        );
        telemetry.addData("Step-up Release Reverse", intakeSubsystem.isStepUpReverseActive());
    }
    /**
     * Gamepad 2 X starts the flywheel; Y stops it.
     * setVelocity() uses encoder ticks/second and the PIDF motor controller.
     * AprilTag velocity adjustment is not active yet.
     */
    public void launch() {
        double targetVelocity = LauncherSubsystem.TELEOP_TARGET_VELOCITY;
        if (gamepad2.x) {
            flywheelEnabled = true;
        } else if (gamepad2.y) {
            flywheelEnabled = false;
        }
        // Reissue the commanded speed while enabled so Dashboard changes
        // take effect immediately; Y still stops the motor as before.
        if (flywheelEnabled) {
            actuator.launchMotor(targetVelocity);
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

        // Ready means measured speed is within the tolerance in Actuators.
        // LED position numbers are device-specific PWM commands, not brightness.
        if (launcherReady) {
            // Flash green: 300ms ON, 150ms OFF
            if ((ledTimer.milliseconds() % 300) < 150) {
                actuator.ledIndicator.setPosition(0.50); // Green
            } else {
                actuator.ledIndicator.setPosition(0.0);  // OFF
            }
        } else {
            actuator.ledIndicator.setPosition(0.28); // Solid red
        }
    }
    /** Display the current state of the non-blocking four-ball sequence. */
    public void launcherTelemetry() {
        telemetry.addData("Distance (cm)", launcher.getLastDistanceCm());
        telemetry.addData("Detecting Empty", launcher.isDetectingEmpty());
        telemetry.addData("Empty Timer", launcher.getEmptyDuration());
        telemetry.addData("Launch State", launcher.getState());
    }

    /** Show fresh AprilTag IDs and angles; telemetry does NOT aim the robot. */
    public void limelightTelemetry() {
        LLResult result = sensor.getFreshAprilTagResult();
        if (result != null) {
            telemetry.addData("Limelight Age (ms)", result.getStaleness());
            telemetry.addData("AprilTags Seen", result.getFiducialResults().size());
            for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
                telemetry.addData("AprilTag " + tag.getFiducialId(),
                        "TX: %.2f deg, TY: %.2f deg",
                        tag.getTargetXDegrees(), tag.getTargetYDegrees());
            }
        } else {
            telemetry.addData("Limelight AprilTags", "No fresh AprilTag detection");
        }
    }
}
