package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.subsystems.Actuators;
import org.firstinspires.ftc.teamcode.subsystems.AprilTagVelocityController;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Sensors;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * STUDENT GUIDE: Driver-controlled robot program (TeleOp).
 * Robot owns the shared hardware and subsystems; this class maps gamepad
 * inputs to commands and displays information on the Driver Station.
 * The while loop must run repeatedly so driving, timers and sensors update.
 */
@Config
@Disabled // Shared implementation; choose TeleOp RED or TeleOp BLUE.
public class TeleOperated extends LinearOpMode {

    // References to the SAME objects created by Robot; no extra hardware setup.
    Robot robot;
    Actuators actuator;
    Sensors sensor;
    IntakeSubsystem intakeSubsystem;
    DrivetrainSubsystem drivetrain;

    // Shared baseline for TeleOp and Auto. TeleOp may override it temporarily
    // using calibrated alliance AprilTags; Auto still uses the baseline.
    // Remember whether X started the flywheel so changes apply while spinning.
    boolean flywheelEnabled = false;
    // Rumble only once each time X starts the flywheel; Y rearms it.
    private boolean flywheelReadyRumbled = false;
    // Actual encoder velocity (ticks/second), refreshed every loop.
    double currentVelocity = 0;
    // Readiness is based on the currently commanded target, not the baseline.
    boolean launcherReady = false;

    // Count each ball once as it passes the intake-to-transfer color sensor.
    public static double BALL_ALPHA_ON = 0.40;
    public static double BALL_ALPHA_OFF = 0.15;
    public static double BALL_DEBOUNCE_SECONDS = 0.06;
    private int ballsCounted = 0;
    private boolean ballAtSensor = false;
    private boolean launchBumperWasPressed = false;
    private final ElapsedTime ballTransitionTimer = new ElapsedTime();
    private final ElapsedTime fullAlertTimer = new ElapsedTime();
    private boolean fullAlertActive = false;
    private double lastBallAlpha = Double.NaN;

    LauncherSubsystem launcher;
    private AprilTagVelocityController visionVelocity;
    private LLResult currentAprilTagFrame;

    /** Overridden by the RED and BLUE OpMode entry points. */
    protected boolean isRedAlliance() { return true; }

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
        visionVelocity = new AprilTagVelocityController(isRedAlliance());

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
                // Read Limelight once per loop for both velocity and telemetry.
                currentAprilTagFrame = sensor.getFreshAprilTagResult();
                launch();
                updateBallCounter();
                updateFullAlertLed();

                // Gamepad 2 right bumper begins the automatic four-ball feed.
                // launcher.start() ignores new starts while already busy.
                if (gamepad2.right_bumper) {
                    if (!launchBumperWasPressed && !launcher.isBusy()) {
                        resetBallCounter();
                    }
                    launcher.start();
                }
                launchBumperWasPressed = gamepad2.right_bumper;
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
    }
    /**
     * Gamepad 2 X starts the flywheel; Y stops it.
     * setVelocity() uses encoder ticks/second and the PIDF motor controller.
     * Only alliance-matching, calibrated tags can adjust the target.
     */
    public void launch() {
        double targetVelocity = LauncherSubsystem.TARGET_VELOCITY;
        if (AprilTagVelocityController.ENABLED) {
            targetVelocity = visionVelocity.update(currentAprilTagFrame,
                    targetVelocity, launcher.isBusy());
        }
        if (gamepad2.x) {
            if (!flywheelEnabled) flywheelReadyRumbled = false;
            flywheelEnabled = true;
        } else if (gamepad2.y) {
            flywheelEnabled = false;
            flywheelReadyRumbled = false;
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

        // A short one-time signal when the flywheel first reaches target speed.
        if (flywheelEnabled && launcherReady && !flywheelReadyRumbled) {
            gamepad2.rumble(300);
            flywheelReadyRumbled = true;
        }

        //Display launcher telemetry
        telemetry.addData("Target Velocity", targetVelocity);
        telemetry.addData("Actual Velocity", currentVelocity);
        telemetry.addData("Velocity Error", flywheelEnabled ? targetVelocity - currentVelocity : 0);
        telemetry.addData("Launcher Ready", launcherReady);
        telemetry.addData("Vision Velocity", AprilTagVelocityController.ENABLED
                && visionVelocity.hasRecentCalibration() ? "Calibrated" : "Baseline / hold");
        telemetry.addData("Calibrated Tags", visionVelocity.getCalibratedTagsSeen());

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
    /** Clear the intake passage count when a new volley begins. */
    private void resetBallCounter() {
        ballsCounted = 0;
        ballAtSensor = false;
        fullAlertActive = false;
        ballTransitionTimer.reset();
    }

    /** Count rising ball detections, not every loop while a ball is present. */
    private void updateBallCounter() {
        lastBallAlpha = sensor.getIntakeColorAlpha();
        if (Double.isFinite(lastBallAlpha) &&
                ballTransitionTimer.seconds() >= BALL_DEBOUNCE_SECONDS) {
            if (!ballAtSensor && lastBallAlpha >= BALL_ALPHA_ON) {
                ballAtSensor = true;
                ballsCounted++;
                ballTransitionTimer.reset();
                if (ballsCounted == 4) {
                    fullAlertActive = true;
                    fullAlertTimer.reset();
                }
            } else if (ballAtSensor && lastBallAlpha <= BALL_ALPHA_OFF) {
                ballAtSensor = false;
                ballTransitionTimer.reset();
            }
        }
        telemetry.addData("Balls Counted", ballsCounted);
        telemetry.addData("Intake Color Alpha", lastBallAlpha);
    }

    /** Temporarily override the usual red/green LED with three yellow flashes. */
    private void updateFullAlertLed() {
        if (!fullAlertActive) return;
        double elapsed = fullAlertTimer.seconds();
        if (elapsed >= 0.6) {
            fullAlertActive = false;
            return;
        }
        // Three quick flashes: 100 ms yellow, 100 ms off each.
        actuator.ledIndicator.setPosition(((int) (elapsed / 0.1)) % 2 == 0
                ? 0.388 : 0.0);
    }

    /** Display the current state of the non-blocking four-ball sequence. */
    public void launcherTelemetry() {
        // Read the sensor every TeleOp loop, even when the launcher is idle.
        // This is telemetry-only; the launcher state machine still controls the kicker.
        telemetry.addData("Distance (cm)", sensor.getDistance(DistanceUnit.CM));
        telemetry.addData("Detecting Empty", launcher.isDetectingEmpty());
        telemetry.addData("Launch State", launcher.getState());
    }

    /** Let experimental child OpModes reuse the already-validated camera frame. */
    protected LLResult getTurretAprilTagFrame() { return currentAprilTagFrame; }

    /** Show only this alliance's fresh AprilTags; the velocity estimator runs separately. */
    public void limelightTelemetry() {
        LLResult result = currentAprilTagFrame;
        if (result != null) {
            telemetry.addData("Limelight Age (ms)", result.getStaleness());
            int allianceTags = 0;
            for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
                if (visionVelocity.acceptsTag(tag.getFiducialId())) allianceTags++;
            }
            telemetry.addData("Alliance AprilTags Seen", allianceTags);
            for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
                if (!visionVelocity.acceptsTag(tag.getFiducialId())) continue;
                telemetry.addData("AprilTag " + tag.getFiducialId(),
                        "TX: %.2f deg, TY: %.2f deg",
                        tag.getTargetXDegrees(), tag.getTargetYDegrees());
            }
        } else {
            telemetry.addData("Limelight AprilTags", "No fresh AprilTag detection");
        }
    }
}
