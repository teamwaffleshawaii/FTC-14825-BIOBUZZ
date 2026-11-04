// Red Right autonomous: Road Runner odometry paths and shared launcher.
// Verify the starting pose and timed feeding on the physical field.

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
import org.firstinspires.ftc.teamcode.subsystems.AutoVisionLaunchAction;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherAction;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Sensors;

/**
 * STUDENT GUIDE: Right-side autonomous route using Road Runner odometry.
 * A Pose2d contains field X, Y and robot heading (in radians).
 * Trajectories below are built BEFORE PLAY and run in a planned sequence.
 * Verify all coordinates against the actual field before a match.
 */
@Autonomous (name = "Red Right", group = "Auto")
public class RedRight extends LinearOpMode {

    //Include all of your actuators
    Robot robot;
    Actuators actuator;
    Sensors sensor;
    IntakeSubsystem intakeSubsystem;
    LauncherSubsystem launcher;

    @Override
    public void runOpMode() throws InterruptedException {
        //Include all of your motors hardware
        robot = new Robot(hardwareMap);
        actuator = robot.actuator;
        sensor = robot.sensor;
        intakeSubsystem = robot.intake;
        launcher = robot.launcher;

        //TODO: Set up Robot starting position here!!!
        // Starting pose is our best estimate of the robot's field position.
        // Road Runner uses it as the reference for subsequent trajectories.
        MecanumDrive drive = new MecanumDrive(hardwareMap,
                new Pose2d(60, -11.5, Math.toRadians(180) ) );

        //TODO: Put your Odometry trajectory paths here!!!
        //Separate your paths by action (linearSlide movements, intake movements, servo movements, etc.)
        // Action builders describe a path; .build() creates an executable action.
        // Road Runner does not move the robot until Actions.runBlocking() runs.
        Action launchPositionRight = drive.actionBuilder(new Pose2d(60, -11.5, Math.toRadians(180) ))
                .strafeToLinearHeading(new Vector2d( 47, -11.5), Math.toRadians(180))
                .build();

        Action pickupGroundPosition = drive.actionBuilder(new Pose2d(47, -11.5, Math.toRadians(180) ))
                .strafeToLinearHeading(new Vector2d( 50, -51), Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d( 57, -51), Math.toRadians(0))
                .build();

        Action launchPosition2 = drive.actionBuilder(new Pose2d(57, -51, Math.toRadians(0) ))
                .strafeToLinearHeading(new Vector2d( 49, -28), Math.toRadians(180))
                .build();

        // Collection path: travel from the first shooting area toward flowers.
        Action flowerPosition = drive.actionBuilder(new Pose2d(49, -28, Math.toRadians(180) ))
                .splineTo(new Vector2d(28, -44), Math.toRadians(270))
                .strafeTo(new Vector2d(19, -60))
                //...
                .build();

        Action moveBack = drive.actionBuilder(new Pose2d(19, -60, Math.toRadians(270) ))
                .strafeToLinearHeading(new Vector2d(25,-44), Math.toRadians(270))
                .build();

        // CHECK: previous path ends heading 90 deg; this starts at 85 deg.
        // Confirm heading continuity before running on the field.
        // Final approach to the second shooting pose.
        Action launchPositionLeft = drive.actionBuilder(new Pose2d(-25, -44, Math.toRadians(270) ))
                .strafeToLinearHeading(new Vector2d(-50,-20), Math.toRadians(5))
                .strafeToLinearHeading(new Vector2d(-50,-15), Math.toRadians(0))
                .build();

        // Parking is a separate trajectory executed after the second volley.
        Action park = drive.actionBuilder(new Pose2d(-50, -15, Math.toRadians(-5) ))
                .strafeToLinearHeading(new Vector2d(-38,-20), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-26,-54), Math.toRadians(90))
                .build();

        //Create more trajectories as needed

        waitForStart();

        // try/finally ensures motors are commanded safe values on exit,
        // including when STOP is pressed or an exception occurs.
        try {
        if (opModeIsActive() && !isStopRequested()) {
            // Spin up while driving to the first launch position.
            // The first volley starts after arriving, regardless of actual speed.
            launcher.setFlywheelVelocity(LauncherSubsystem.TARGET_VELOCITY);
            // runBlocking executes a Road Runner action until it finishes.
            // Do not use a long Thread.sleep() during trajectory following.
            Actions.runBlocking(new SequentialAction(
                    launchPositionRight
            ));

            // Launch regardless of flywheel readiness so the preload balls
            // are cleared before collecting the next four.
            if (opModeIsActive() && !isStopRequested()) {
                // LauncherAction runs the timed feed state machine, not a drive path.
                // Sample the local alliance tags, lock the speed, then feed balls.
                    Actions.runBlocking(new AutoVisionLaunchAction(sensor, launcher, actuator, true));
                    Actions.runBlocking(new LauncherAction(launcher));
            }

            // Stop the flywheel regardless of whether the first shot succeeded.
            launcher.setFlywheelVelocity(0);

            if (opModeIsActive() && !isStopRequested()) {
                // Go to the corner to pick up pollen
                intakeSubsystem.intake(1);
                Actions.runBlocking(new SequentialAction(
                        pickupGroundPosition,
                        // Road Runner timed pause to allow collection.
                        // Unlike Thread.sleep(), this is part of the action sequence.
                        new SleepAction(3),
                        // Get ready to launch again
                        launchPosition2
                ));

                //Stop the intake, reverse the step-up a little
                intakeSubsystem.stopIntake();
                intakeSubsystem.startStepUpReversePulse();
                Actions.runBlocking((packet) -> {
                    intakeSubsystem.update();
                    return intakeSubsystem.isStepUpReverseActive();
                });
                intakeSubsystem.stopFeeding();
            }

            //launch the ground pollen
            if (opModeIsActive() && !isStopRequested()) {
                // Spin up for the second volley on the way to the launch location.
                launcher.setFlywheelVelocity(LauncherSubsystem.TARGET_VELOCITY);
                // Launch regardless of flywheel readiness, as with the first volley.
                if (opModeIsActive() && !isStopRequested()) {
                    // Sample the local alliance tags, lock the speed, then feed balls.
                    Actions.runBlocking(new AutoVisionLaunchAction(sensor, launcher, actuator, true));
                    Actions.runBlocking(new LauncherAction(launcher));
                }
                launcher.setFlywheelVelocity(0);
            }

            //Go to the flower
            if (opModeIsActive() && !isStopRequested()) {
                // Both intake() motors (intake and transfer) run during collection.
                intakeSubsystem.rampDown();
                intakeSubsystem.intake(1);
                Actions.runBlocking(new SequentialAction(
                        flowerPosition,
                        // Road Runner timed pause to allow collection.
                        // Unlike Thread.sleep(), this is part of the action sequence.
                        new SleepAction(3),
                        // Reverse out of flowers before raising ramp
                        moveBack
                ));

                // After backing out of flowers, raise ramp, stop intake, and reverse transfer
                intakeSubsystem.rampUp();
                intakeSubsystem.stopIntake();
                intakeSubsystem.startStepUpReversePulse();
                Actions.runBlocking((packet) -> {
                    intakeSubsystem.update();
                    return intakeSubsystem.isStepUpReverseActive();
                });
                intakeSubsystem.stopFeeding();
            }

            if (opModeIsActive() && !isStopRequested()) {
                // Spin up for the second volley on the way to the launch location.
                launcher.setFlywheelVelocity(LauncherSubsystem.TARGET_VELOCITY);
                Actions.runBlocking(launchPositionLeft);
                // Launch regardless of flywheel readiness, as with the first volley.
                if (opModeIsActive() && !isStopRequested()) {
                    // Sample the local alliance tags, lock the speed, then feed balls.
                    Actions.runBlocking(new AutoVisionLaunchAction(sensor, launcher, actuator, true));
                    Actions.runBlocking(new LauncherAction(launcher));
                }
                launcher.setFlywheelVelocity(0);
            }

            // Park after autonomous sequence.
            if (opModeIsActive() && !isStopRequested()) {
                Actions.runBlocking(new SequentialAction(
                        park));
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