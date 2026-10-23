// Blue Left autonomous using Road Runner odometry.
// Verify starting pose, headings, and ball-feed timing on the real field.

package org.firstinspires.ftc.teamcode;

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
        // Road Runner does not move the robot until Actions.runBlocking() runs.
        Action launchPositionRight = drive.actionBuilder(new Pose2d(60, 11.5, Math.toRadians(180) ))
                .strafeToLinearHeading(new Vector2d( 47, 11.5), Math.toRadians(180))
                .build();

        // Collection path: travel from the first shooting area toward flowers.
        Action flowerPosition1 = drive.actionBuilder(new Pose2d(47, 11.5, Math.toRadians(180) ))
                .splineTo(new Vector2d(38, 33), Math.toRadians(0))
                //...
                .build();

        // Continue through the collection area before the timed pause.
        Action flowerPosition2 = drive.actionBuilder(new Pose2d(38, 33, Math.toRadians(0) ))
                .strafeTo(new Vector2d(56, 33))
                .build();

        Action moveBack = drive.actionBuilder(new Pose2d(56, 33, Math.toRadians(0) ))
                .strafeToLinearHeading(new Vector2d(40,33), Math.toRadians(0))
                .build();

        // Return to the second shooting position (same side of field).
        Action movetolaunchPositionAgain = drive.actionBuilder(new Pose2d(40, 33, Math.toRadians(0) ))
                .strafeToLinearHeading(new Vector2d(47,12), Math.toRadians(175))
                .build();

        // Parking is a separate trajectory executed after the second volley.
        Action park = drive.actionBuilder(new Pose2d(47, 12, Math.toRadians(175) ))
                .strafeToLinearHeading(new Vector2d(51,65), Math.toRadians(270))
                .build();

        //Create more trajectories as needed

        waitForStart();

        // try/finally ensures motors are commanded safe values on exit,
        // including when STOP is pressed or an exception occurs.
        try {
        if (opModeIsActive() && !isStopRequested()) {

            // Optional start delay for strategy (currently disabled).
            // Actions.runBlocking(new SleepAction(8));

            // Spin up while driving to the first launch position.
            // The first volley starts after arriving, regardless of actual speed.
            launcher.setFlywheelVelocity(launcher.targetVelocity);
            // runBlocking executes a Road Runner action until it finishes.
            // Do not use a long Thread.sleep() during trajectory following.
            Actions.runBlocking(new SequentialAction(
                    launchPositionRight,
                    new SleepAction(1)
            ));

            // Launch regardless of flywheel readiness so the preload balls
            // are cleared before collecting the next four.
            if (opModeIsActive() && !isStopRequested()) {
                // LauncherAction runs the timed feed state machine, not a drive path.
                Actions.runBlocking(new LauncherAction(launcher));
            }

            // Stop the flywheel regardless of whether the first shot succeeded.
            launcher.setFlywheelVelocity(0);

            if (opModeIsActive() && !isStopRequested()) {
                // Both intake() motors (intake and transfer) run during collection.
                intakeSubsystem.rampDown();
                intakeSubsystem.intake(1);
                    Actions.runBlocking(new SequentialAction(
                            flowerPosition1,
                            flowerPosition2,
                            // Road Runner timed pause to allow collection.
                            // Unlike Thread.sleep(), this is part of the action sequence.
                            new SleepAction(3.5)
                    ));

            }

            if (opModeIsActive() && !isStopRequested()) {
                // CAUTION: intake is still running during both return paths.
                // If balls creep toward the launcher, stop and retract earlier.
                Actions.runBlocking(new SequentialAction(
                        moveBack,
                        movetolaunchPositionAgain ));
                intakeSubsystem.rampUp();
                intakeSubsystem.stopIntake();

                // Same short retract pulse used when releasing intake in TeleOp.
                // Back the leading balls away from the launcher before the next volley.
                intakeSubsystem.startStepUpReversePulse();
                Actions.runBlocking((packet) -> {
                    intakeSubsystem.update();
                    return intakeSubsystem.isStepUpReverseActive();
                });
                // update() stops the step-ups; also stop the transfer motor.
                intakeSubsystem.stopFeeding();

                // This is a post-retract wait, not additional retract time:
                // the reverse pulse has already ended and all feed motors are stopped.
                Actions.runBlocking(new SequentialAction(new SleepAction(1)));

                // Second flywheel starts only here, immediately before feeding.
                // Unlike the first volley, there is no drive or warm-up delay.
                // Check measured flywheel speed if the second volley is inconsistent.
                launcher.setFlywheelVelocity(launcher.targetVelocity);

                // LauncherAction uses the autonomous-only 0.04 s empty delay.
                // Launch regardless of flywheel readiness, as with the first volley.
                if (opModeIsActive() && !isStopRequested()) {
                    Actions.runBlocking(new LauncherAction(launcher));
                }
                launcher.setFlywheelVelocity(0);

                // Park after the second launch.
                if (opModeIsActive() && !isStopRequested()) {
                    Actions.runBlocking(new SequentialAction(
                            park ));
                }
            }
        }
        // Cleanup executes whether autonomous finishes normally or stops early.
        } finally {
            launcher.setFlywheelVelocity(0);
            intakeSubsystem.rampUp();
            robot.shutdown();
        }
    }

}