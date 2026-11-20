// Red Right autonomous: Road Runner odometry paths and shared launcher.
// Verify the starting pose and timed feeding on the physical field.

package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import java.util.Arrays;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.odometry.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Actuators;
import org.firstinspires.ftc.teamcode.subsystems.AutoLedAction;
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
        // Road Runner does not move the robot until AutoLedAction.runBlocking(actuator, launcher, ) runs.
        Action launchPositionRight = drive.actionBuilder(new Pose2d(60, -11.5, Math.toRadians(180) ))
                .strafeToLinearHeading(new Vector2d( 47, -31), Math.toRadians(160))
                .build();

        Action pickupGroundPosition = drive.actionBuilder(new Pose2d(47, -31, Math.toRadians(160) ))
                .strafeToLinearHeading(new Vector2d( 50, -51), Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d( 60, -54), Math.toRadians(0))
                .build();

        Action launchPosition2 = drive.actionBuilder(new Pose2d(60, -54, Math.toRadians(0) ))
                .strafeToLinearHeading(new Vector2d( 52, -51), Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d( 49, -31), Math.toRadians(180))
                .build();

        // Collection path: travel from the first shooting area toward flowers.
        Action flowerPosition = drive.actionBuilder(new Pose2d(49, -31, Math.toRadians(180) ))
                .splineTo(new Vector2d(28, -42), Math.toRadians(270))
                .strafeTo(new Vector2d(23, -55))
                //...
                .build();

        Action moveBack = drive.actionBuilder(new Pose2d(23, -55, Math.toRadians(270) ))
                .strafeToLinearHeading(new Vector2d(28,-42), Math.toRadians(270))
                .build();

        // Speed up the crossing across the field
        VelConstraint fastVel = new MinVelConstraint(Arrays.asList(
                drive.kinematics.new WheelVelConstraint(95),
                new AngularVelConstraint(Math.PI * 1.5)
        ));
        AccelConstraint fastAccel = new ProfileAccelConstraint(-75, 90);

        // Final approach to the second shooting pose (fast crossing via Y=-38 across mid-field to clear the hive).
        Action launchPositionLeft = drive.actionBuilder(new Pose2d(28, -42, Math.toRadians(270) ))
                .strafeToLinearHeading(new Vector2d(0, -38), Math.toRadians(0), fastVel, fastAccel)
                .strafeToLinearHeading(new Vector2d(-45, -25), Math.toRadians(20), fastVel, fastAccel)
                .build();

        // Direct approach to second shooting pose if skipping flower (fast crossing via Y=-38 across mid-field to clear the hive).
        Action launchPositionLeftFromGround = drive.actionBuilder(new Pose2d(49, -25, Math.toRadians(180) ))
                .strafeToLinearHeading(new Vector2d(0, -33), Math.toRadians(0), fastVel, fastAccel)
                .strafeToLinearHeading(new Vector2d(-55, -25), Math.toRadians(17), fastVel, fastAccel)
                .build();

        // Left-side flower collection (behind the left shooting pose)
        Action flowerPositionLeft = drive.actionBuilder(new Pose2d(-55, -25, Math.toRadians(17) ))
                .splineTo(new Vector2d(-38, -30), Math.toRadians(180))
                .strafeTo(new Vector2d(-66, -21))
                .build();

        Action moveBackLeft = drive.actionBuilder(new Pose2d(-66, -21, Math.toRadians(180) ))
                .strafeToLinearHeading(new Vector2d(-40, -30), Math.toRadians(180))
                .build();

        Action movetolaunchPositionAgainLeft = drive.actionBuilder(new Pose2d(-40, -30, Math.toRadians(180) ))
                .strafeToLinearHeading(new Vector2d(-45, -10), Math.toRadians(-1))
                .strafeToLinearHeading(new Vector2d(-55, -25), Math.toRadians(17))
                .build();

        // Parking is a separate trajectory executed after the left-side volley.
        Action park = drive.actionBuilder(new Pose2d(-55, -25, Math.toRadians(17) ))
                .strafeToLinearHeading(new Vector2d(-30, -40), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-25, -51), Math.toRadians(90))
                .build();

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
            AutoLedAction.runBlocking(actuator, launcher, new SequentialAction(
                    launchPositionRight
            ));

            // Launch regardless of flywheel readiness so the preload balls
            // are cleared before collecting the next four.
            if (opModeIsActive() && !isStopRequested()) {
                // LauncherAction runs the timed feed state machine, not a drive path.
                // Sample the local alliance tags, lock the speed, then feed balls.
                    AutoLedAction.runBlocking(actuator, launcher, new AutoVisionLaunchAction(sensor, launcher, actuator, true));
                    AutoLedAction.runBlocking(actuator, launcher, new LauncherAction(launcher));
            }

            // Stop the flywheel regardless of whether the first shot succeeded.
            launcher.setFlywheelVelocity(0);

            if (opModeIsActive() && !isStopRequested()) {
                // Go to the corner to pick up pollen
                intakeSubsystem.intake(1);
                AutoLedAction.runBlocking(actuator, launcher, new SequentialAction(
                        pickupGroundPosition,
                        new SleepAction(1.2),
                        launchPosition2
                ));

                //Stop the intake, reverse the step-up a little
                intakeSubsystem.stopIntake();
                intakeSubsystem.startStepUpReversePulse();
                AutoLedAction.runBlocking(actuator, launcher, (packet) -> {
                    intakeSubsystem.update();
                    return intakeSubsystem.isStepUpReverseActive();
                });
                intakeSubsystem.stopFeeding();
            }

            // After ground pickup, check if Limelight sees the correct red alliance AprilTag (IDs 30-37)
            boolean seesAllianceTag = sensor.seesAllianceAprilTag(true, 0.1);

            if (seesAllianceTag) {
                // launch the ground pollen
                if (opModeIsActive() && !isStopRequested()) {
                    launcher.setFlywheelVelocity(LauncherSubsystem.TARGET_VELOCITY);
                    if (opModeIsActive() && !isStopRequested()) {
                        AutoLedAction.runBlocking(actuator, launcher, new AutoVisionLaunchAction(sensor, launcher, actuator, true));
                        AutoLedAction.runBlocking(actuator, launcher, new LauncherAction(launcher));
                    }
                    launcher.setFlywheelVelocity(0);
                }

                // Go to the flower
                if (opModeIsActive() && !isStopRequested()) {
                    intakeSubsystem.rampDown();
                    intakeSubsystem.intake(1);
                    AutoLedAction.runBlocking(actuator, launcher, new SequentialAction(
                            flowerPosition,
                            new SleepAction(1.2),
                            moveBack
                    ));
                }

                if (opModeIsActive() && !isStopRequested()) {
                    // Spin up flywheel while crossing to the left side
                    launcher.setFlywheelVelocity(LauncherSubsystem.TARGET_VELOCITY);

                    AutoLedAction.runBlocking(actuator, launcher, launchPositionLeft);
                }
            } else {
                // Limelight does not see red alliance AprilTag:
                // Skip ground launch and flower pickup, head straight to the other side!
                if (opModeIsActive() && !isStopRequested()) {
                    // Spin up flywheel while crossing to the left side
                    launcher.setFlywheelVelocity(LauncherSubsystem.TARGET_VELOCITY);

                    AutoLedAction.runBlocking(actuator, launcher, launchPositionLeftFromGround);
                }
            }

            // Once arrived on the left side: launch, pick up from left flower behind it, and park!
            if (opModeIsActive() && !isStopRequested()) {
                // Raise ramp, stop intake, and reverse transfer from crossing
                intakeSubsystem.rampUp();
                intakeSubsystem.stopIntake();
                intakeSubsystem.startStepUpReversePulse();
                AutoLedAction.runBlocking(actuator, launcher, (packet) -> {
                    intakeSubsystem.update();
                    return intakeSubsystem.isStepUpReverseActive();
                });
                intakeSubsystem.stopFeeding();

                // Launch on left side
                if (opModeIsActive() && !isStopRequested()) {
                    AutoLedAction.runBlocking(actuator, launcher, new AutoVisionLaunchAction(sensor, launcher, actuator, true));
                    AutoLedAction.runBlocking(actuator, launcher, new LauncherAction(launcher));
                }
                launcher.setFlywheelVelocity(0);

                // Collect from the left-side flower behind the robot
                if (opModeIsActive() && !isStopRequested()) {
                    intakeSubsystem.rampDown();
                    intakeSubsystem.intake(1);
                    AutoLedAction.runBlocking(actuator, launcher, new SequentialAction(
                            flowerPositionLeft,
                            new SleepAction(1.2),
                            moveBackLeft
                    ));

                    // After backing out of left flower, raise ramp, stop intake, and reverse transfer
                    intakeSubsystem.rampUp();
                    intakeSubsystem.stopIntake();
                    intakeSubsystem.startStepUpReversePulse();
                    AutoLedAction.runBlocking(actuator, launcher, (packet) -> {
                        intakeSubsystem.update();
                        return intakeSubsystem.isStepUpReverseActive();
                    });
                    intakeSubsystem.stopFeeding();
                }

                // Move to left launch position again and launch left flower balls
                if (opModeIsActive() && !isStopRequested()) {
                    launcher.setFlywheelVelocity(LauncherSubsystem.TARGET_VELOCITY);
                    AutoLedAction.runBlocking(actuator, launcher, movetolaunchPositionAgainLeft);

                    launcher.requireBallBeforeKicker = true;

                    if (opModeIsActive() && !isStopRequested()) {
                        AutoLedAction.runBlocking(actuator, launcher, new AutoVisionLaunchAction(sensor, launcher, actuator, true));
                        AutoLedAction.runBlocking(actuator, launcher, new LauncherAction(launcher));
                    }
                    launcher.setFlywheelVelocity(0);
                }
            }

            // Park after picking up from left flower.
            if (opModeIsActive() && !isStopRequested()) {
                AutoLedAction.runBlocking(actuator, launcher, new SequentialAction(
                        park));
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