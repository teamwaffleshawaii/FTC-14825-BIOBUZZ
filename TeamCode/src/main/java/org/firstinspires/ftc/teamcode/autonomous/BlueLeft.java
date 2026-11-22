
// Blue Left autonomous using Road Runner odometry.
// Waits up to 11 seconds for a Blue Alliance AprilTag.
// If no tag is detected, the robot skips the normal autonomous and parks.

package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.odometry.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Actuators;
import org.firstinspires.ftc.teamcode.subsystems.AutoLedAction;
import org.firstinspires.ftc.teamcode.subsystems.AutoVisionLaunchAction;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherAction;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Sensors;

@Autonomous(name = "Blue Left", group = "Auto")
public class BlueLeft extends LinearOpMode {

    // References to shared Robot subsystems.
    Robot robot;
    Actuators actuator;
    Sensors sensor;
    IntakeSubsystem intakeSubsystem;
    LauncherSubsystem launcher;

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize robot hardware and subsystems.
        robot = new Robot(hardwareMap);
        actuator = robot.actuator;
        sensor = robot.sensor;
        intakeSubsystem = robot.intake;
        launcher = robot.launcher;

        // Starting pose.
        Pose2d startingPose = new Pose2d(
                60, 11.5, Math.toRadians(180)
        );

        MecanumDrive drive = new MecanumDrive(hardwareMap, startingPose);

        // First shooting position.
        Action launchPositionRight = drive.actionBuilder(
                        new Pose2d(60, 11.5, Math.toRadians(180)))
                .strafeToLinearHeading(
                        new Vector2d(47, 11.5), Math.toRadians(180))
                .strafeToLinearHeading(
                        new Vector2d(46, 32), Math.toRadians(200))
                .build();

        // Collection path.
        Action flowerPosition = drive.actionBuilder(
                        new Pose2d(46, 32, Math.toRadians(200)))
                .splineTo(
                        new Vector2d(38, 33), Math.toRadians(0))
                .strafeTo(
                        new Vector2d(58, 30))
                .build();

        // Move backward out of the collection area.
        Action moveBack = drive.actionBuilder(
                        new Pose2d(58, 30, Math.toRadians(0)))
                .strafeToLinearHeading(
                        new Vector2d(40, 33), Math.toRadians(0))
                .build();

        // Return to the second shooting position.
        Action movetolaunchPositionAgain = drive.actionBuilder(
                        new Pose2d(40, 33, Math.toRadians(0)))
                .strafeToLinearHeading(
                        new Vector2d(47, 12), Math.toRadians(179))
                .strafeToLinearHeading(
                        new Vector2d(46, 32), Math.toRadians(200))
                .build();

        // Normal parking path after completing the autonomous.
        Action park = drive.actionBuilder(
                        new Pose2d(46, 32, Math.toRadians(200)))
                .strafeToLinearHeading(
                        new Vector2d(50, 60), Math.toRadians(270))
                .build();

        // Emergency parking path if no AprilTag is detected.
        // This path starts at the robot's actual starting pose.
        Action parkFromStart = drive.actionBuilder(
                        new Pose2d(60, 11.5, Math.toRadians(180)))
                .strafeToLinearHeading(
                        new Vector2d(50, 60), Math.toRadians(270))
                .build();

        waitForStart();

        try {
            if (opModeIsActive() && !isStopRequested()) {

                // ==========================================
                // WAIT UP TO 11 SECONDS FOR BLUE APRILTAG
                // ==========================================

                final double TAG_WAIT_SECONDS = 11.0;

                boolean aprilTagDetected = false;

                long tagWaitStartTime = System.nanoTime();

                telemetry.addLine("WAITING FOR BLUE ALLIANCE APRILTAG");
                telemetry.addData("Time limit", "%.0f seconds", TAG_WAIT_SECONDS);
                telemetry.update();

                while (opModeIsActive()
                        && !isStopRequested()
                        && !aprilTagDetected
                        && (System.nanoTime() - tagWaitStartTime)
                        / 1_000_000_000.0 < TAG_WAIT_SECONDS) {

                    // Check for a Blue Alliance AprilTag.
                    aprilTagDetected = sensor.seesAllianceAprilTag(false, 0.5);

                    double elapsedSeconds =
                            (System.nanoTime() - tagWaitStartTime)
                                    / 1_000_000_000.0;

                    double remainingSeconds =
                            TAG_WAIT_SECONDS - elapsedSeconds;

                    telemetry.clearAll();
                    telemetry.addLine("WAITING FOR BLUE ALLIANCE APRILTAG");
                    telemetry.addData("Tag detected", aprilTagDetected);
                    telemetry.addData("Time remaining", "%.1f seconds",
                            Math.max(0, remainingSeconds));
                    telemetry.update();

                    idle();
                }

                // Stop immediately if STOP was pressed.
                if (!opModeIsActive() || isStopRequested()) {
                    return;
                }

                // ==========================================
                // NO TAG FOUND: SKIP NORMAL AUTO AND PARK
                // ==========================================

                if (!aprilTagDetected) {

                    telemetry.clearAll();
                    telemetry.addLine("NO APRILTAG DETECTED");
                    telemetry.addLine("11 SECOND TIMEOUT");
                    telemetry.addLine("SKIPPING NORMAL AUTONOMOUS");
                    telemetry.addLine("RUNNING PARKING PATH");
                    telemetry.update();

                    // Make sure the flywheel and intake are stopped.
                    launcher.setFlywheelVelocity(0);
                    intakeSubsystem.stopIntake();
                    intakeSubsystem.stopFeeding();

                    if (opModeIsActive() && !isStopRequested()) {
                        AutoLedAction.runBlocking(
                                actuator,
                                launcher,
                                new SequentialAction(parkFromStart)
                        );
                    }

                } else {

                    // ==========================================
                    // APRILTAG FOUND: RUN NORMAL AUTONOMOUS
                    // ==========================================

                    telemetry.clearAll();
                    telemetry.addLine("BLUE APRILTAG DETECTED!");
                    telemetry.addLine("STARTING NORMAL AUTONOMOUS");
                    telemetry.update();

                    // Wait one second before beginning the paths.
                    AutoLedAction.runBlocking(
                            actuator,
                            launcher,
                            new SleepAction(1)
                    );

                    // Spin up while driving to the first launch position.
                    launcher.setFlywheelVelocity(
                            LauncherSubsystem.TARGET_VELOCITY
                    );

                    AutoLedAction.runBlocking(
                            actuator,
                            launcher,
                            new SequentialAction(
                                    launchPositionRight
                            )
                    );

                    // Launch preload balls.
                    if (opModeIsActive() && !isStopRequested()) {

                        AutoLedAction.runBlocking(
                                actuator,
                                launcher,
                                new AutoVisionLaunchAction(
                                        sensor, launcher, actuator, false
                                )
                        );

                        AutoLedAction.runBlocking(
                                actuator,
                                launcher,
                                new LauncherAction(launcher)
                        );
                    }

                    // Stop the flywheel after the first volley.
                    launcher.setFlywheelVelocity(0);

                    // Collect balls.
                    if (opModeIsActive() && !isStopRequested()) {

                        intakeSubsystem.rampDown();
                        intakeSubsystem.intake(1);

                        AutoLedAction.runBlocking(
                                actuator,
                                launcher,
                                new SequentialAction(
                                        flowerPosition,
                                        new SleepAction(2),
                                        moveBack
                                )
                        );

                        // Raise ramp, stop intake, and reverse transfer.
                        intakeSubsystem.rampUp();
                        intakeSubsystem.stopIntake();
                        intakeSubsystem.startStepUpReversePulse();

                        AutoLedAction.runBlocking(
                                actuator,
                                launcher,
                                (packet) -> {
                                    intakeSubsystem.update();
                                    return intakeSubsystem.isStepUpReverseActive();
                                }
                        );

                        intakeSubsystem.stopFeeding();
                    }

                    // Return to the second shooting position.
                    if (opModeIsActive() && !isStopRequested()) {

                        launcher.setFlywheelVelocity(
                                LauncherSubsystem.TARGET_VELOCITY
                        );

                        AutoLedAction.runBlocking(
                                actuator,
                                launcher,
                                movetolaunchPositionAgain
                        );

                        // Allow the flywheel to settle.
                        if (opModeIsActive() && !isStopRequested()) {
                            AutoLedAction.runBlocking(
                                    actuator,
                                    launcher,
                                    new SleepAction(1)
                            );
                        }

                        // Require a ball before allowing the kicker to rise.
                        launcher.requireBallBeforeKicker = true;

                        // Launch the second volley.
                        if (opModeIsActive() && !isStopRequested()) {

                            AutoLedAction.runBlocking(
                                    actuator,
                                    launcher,
                                    new AutoVisionLaunchAction(
                                            sensor, launcher, actuator, false
                                    )
                            );

                            AutoLedAction.runBlocking(
                                    actuator,
                                    launcher,
                                    new LauncherAction(launcher)
                            );
                        }

                        launcher.setFlywheelVelocity(0);
                    }

                    // Normal parking path after completing autonomous.
                    if (opModeIsActive() && !isStopRequested()) {

                        AutoLedAction.runBlocking(
                                actuator,
                                launcher,
                                new SequentialAction(
                                        park
                                )
                        );
                    }
                }
            }

        } finally {

            // Cleanup whether autonomous finishes or STOP is pressed.
            launcher.setFlywheelVelocity(0);
            intakeSubsystem.rampUp();
            intakeSubsystem.stopIntake();
            intakeSubsystem.stopFeeding();

            robot.shutdown();

            sleep(1000);
        }
    }
}
