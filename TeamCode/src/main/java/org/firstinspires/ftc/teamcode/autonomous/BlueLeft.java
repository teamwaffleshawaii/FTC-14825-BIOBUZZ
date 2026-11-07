// Blue Left autonomous using Road Runner odometry.
// Verify starting pose, headings, and ball-feed timing on the real field.

package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
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

/**
 * STUDENT GUIDE: Left-side autonomous route using Road Runner odometry.
 * A Pose2d contains field X, Y and robot heading (in radians).
 * Trajectories below are built BEFORE PLAY and run in a planned sequence.
 * Verify all coordinates against the actual field before a match.
 */
@Autonomous (name = "Blue Left", group = "Auto")
public class BlueLeft extends LinearOpMode {

    // References to shared Robot subsystems (not duplicate hardware).
    Robot robot;
    Actuators actuator;
    Sensors sensor;
    IntakeSubsystem intakeSubsystem;
    LauncherSubsystem launcher;

    @Override
    public void runOpMode() throws InterruptedException {
        // Robot initializes all shared hardware and subsystems.
        robot = new Robot(hardwareMap);
        actuator = robot.actuator;
        sensor = robot.sensor;
        intakeSubsystem = robot.intake;
        launcher = robot.launcher;

        // Starting pose must match the robot's real position and heading.
        // Starting pose is our best estimate of the robot's field position.
        // Road Runner uses it as the reference for subsequent trajectories.
        MecanumDrive drive = new MecanumDrive(hardwareMap,
                new Pose2d(60, 11.5, Math.toRadians(180) ) );

        // Each action begins at the previous action's expected ending pose.
        // Action builders describe a path; .build() creates an executable action.
        // Road Runner does not move the robot until AutoLedAction.runBlocking(actuator, launcher, ) runs.
        Action launchPositionRight = drive.actionBuilder(new Pose2d(60, 11.5, Math.toRadians(180) ))
                .strafeToLinearHeading(new Vector2d( 47, 11.5), Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d( 46, 32), Math.toRadians(200))
                .build();

        // Collection path: travel from the first shooting area toward flowers.
        Action flowerPosition = drive.actionBuilder(new Pose2d(46, 32, Math.toRadians(200) ))
                .splineTo(new Vector2d(38, 33), Math.toRadians(0))
                .strafeTo(new Vector2d(58, 33))
                .build();

        Action moveBack = drive.actionBuilder(new Pose2d(58, 33, Math.toRadians(0) ))
                .strafeToLinearHeading(new Vector2d(40,33), Math.toRadians(0))
                .build();

        // Return to the second shooting position (same side of field).
        Action movetolaunchPositionAgain = drive.actionBuilder(new Pose2d(40, 33, Math.toRadians(0) ))
                .strafeToLinearHeading(new Vector2d(47,12), Math.toRadians(179))
                .strafeToLinearHeading(new Vector2d(46,32), Math.toRadians(200))
                .build();

        // Parking is a separate trajectory executed after the second volley.
        Action park = drive.actionBuilder(new Pose2d(46, 32, Math.toRadians(184) ))
                .strafeToLinearHeading(new Vector2d(50,60), Math.toRadians(270))
                .build();

        waitForStart();

        // try/finally ensures motors are commanded safe values on exit,
        // including when STOP is pressed or an exception occurs.
        try {
        if (opModeIsActive() && !isStopRequested()) {

            //Wait for x seconds for the partner to score
            new SleepAction(1);

            // Spin up while driving to the first launch position.
            // The first volley starts after arriving, regardless of actual speed.
            launcher.setFlywheelVelocity(LauncherSubsystem.TARGET_VELOCITY);
            // runBlocking executes a Road Runner action until it finishes.
            // Do not use a long Thread.sleep() during trajectory following.
            AutoLedAction.runBlocking(actuator, launcher, new SequentialAction(
                    launchPositionRight
            ));

            // Launch preload balls (first launch uses default forward turret angle)
            if (opModeIsActive() && !isStopRequested()) {
                // LauncherAction runs the timed feed state machine, not a drive path.
                // Sample the local alliance tags, lock the speed, then feed balls.
                AutoLedAction.runBlocking(actuator, launcher, new AutoVisionLaunchAction(sensor, launcher, actuator, false));
                AutoLedAction.runBlocking(actuator, launcher, new LauncherAction(launcher));
            }

            // Stop the flywheel regardless of whether the first shot succeeded.
            launcher.setFlywheelVelocity(0);

            if (opModeIsActive() && !isStopRequested()) {
                // Both intake() motors (intake and transfer) run during collection.
                intakeSubsystem.rampDown();
                intakeSubsystem.intake(1);
                AutoLedAction.runBlocking(actuator, launcher, new SequentialAction(
                        flowerPosition,
                        // Road Runner timed pause to allow collection.
                        // Unlike Thread.sleep(), this is part of the action sequence.
                        new SleepAction(2),
                        // Reverse out of flowers before raising ramp
                        moveBack
                ));

                // After backing out of flowers, raise ramp, stop intake, and reverse transfer
                intakeSubsystem.rampUp();
                intakeSubsystem.stopIntake();
                intakeSubsystem.startStepUpReversePulse();
                AutoLedAction.runBlocking(actuator, launcher, (packet) -> {
                    intakeSubsystem.update();
                    return intakeSubsystem.isStepUpReverseActive();
                });
                intakeSubsystem.stopFeeding();
            }

            if (opModeIsActive() && !isStopRequested()) {
                if (opModeIsActive() && !isStopRequested()) {
                    // Spin up the flywheel during the final approach.
                    launcher.setFlywheelVelocity(LauncherSubsystem.TARGET_VELOCITY);
                    AutoLedAction.runBlocking(actuator, launcher, movetolaunchPositionAgain);
                }

                // Allow the flywheel to settle after retract, before feeding.
                if (opModeIsActive() && !isStopRequested()) {
                    AutoLedAction.runBlocking(actuator, launcher, new SleepAction(1));
                }

                // Uses the shared autonomous 40 ms empty delay.
                // This second volley must first detect a ball
                // before an empty sensor reading is allowed to raise the kicker.
                // This prevents an initially empty sensor from ending the volley.
                launcher.requireBallBeforeKicker = true;

                // LauncherAction feeds the second four balls only after the return
                // drive and spin-up delay. Uses the same 0.04 s delay as Blue Right.
                // Like Blue Right, this does not gate on measured flywheel readiness.
                if (opModeIsActive() && !isStopRequested()) {
                    // Sample the local alliance tags, lock the speed, then feed balls.
                    AutoLedAction.runBlocking(actuator, launcher, new AutoVisionLaunchAction(sensor, launcher, actuator, false));
                    AutoLedAction.runBlocking(actuator, launcher, new LauncherAction(launcher));
                }
                launcher.setFlywheelVelocity(0);
            }

            // Park after autonomous sequence.
            if (opModeIsActive() && !isStopRequested()) {
                AutoLedAction.runBlocking(actuator, launcher, new SequentialAction(
                        park));
            }
        }
        // Cleanup executes whether autonomous finishes normally or stops early.
        } finally {
            launcher.setFlywheelVelocity(0);
            intakeSubsystem.rampUp();
            robot.shutdown();
            sleep(1000);
        }
    }

}