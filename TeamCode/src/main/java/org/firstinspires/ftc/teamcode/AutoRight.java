//Auto Template that you can use to program your auto
// NOTE: THIS CODE RUNS ON ODOMETRY, you must have your odometry tuned

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.odometry.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Actuators;
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
@Autonomous (name = "AutoRight", group = "Auto")
public class AutoRight extends LinearOpMode {

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

        // Collection path: travel from the first shooting area toward flowers.
        Action flowerPosition1 = drive.actionBuilder(new Pose2d(47, -11.5, Math.toRadians(180) ))
                .splineTo(new Vector2d(30, -44), Math.toRadians(270))
                //...
                .build();

        // Continue through the collection area before the timed pause.
        Action flowerPosition2 = drive.actionBuilder(new Pose2d(30, -44, Math.toRadians(270) ))
                .strafeTo(new Vector2d(25, -55))
                .build();

        Action moveBack = drive.actionBuilder(new Pose2d(25, -55, Math.toRadians(270) ))
                .strafeToLinearHeading(new Vector2d(25,-44), Math.toRadians(270))
                .build();

        // Reposition to the left side for the second launch.
        Action movetolaunchPositionLeft = drive.actionBuilder(new Pose2d(25, -44, Math.toRadians(270) ))
                .strafeToLinearHeading(new Vector2d(-30,-58), Math.toRadians(90))
                .build();

        // Final approach to the second shooting pose.
        Action launchPositionLeft = drive.actionBuilder(new Pose2d(-30, -58, Math.toRadians(85) ))
                .strafeToLinearHeading(new Vector2d(-54,-1.5), Math.toRadians(-5))
                .build();

        // Parking is a separate trajectory executed after the second volley.
        Action park1 = drive.actionBuilder(new Pose2d(-54, -1.5, Math.toRadians(-5) ))
                .strafeToLinearHeading(new Vector2d(-30,-46), Math.toRadians(90))
                .build();

        Action park2 = drive.actionBuilder(new Pose2d(-30, -46, Math.toRadians(90) ))
                .strafeToLinearHeading(new Vector2d(-30,-56), Math.toRadians(90))
                .build();

        //Create more trajectories as needed

        waitForStart();

        // try/finally ensures motors are commanded safe values on exit,
        // including when STOP is pressed or an exception occurs.
        try {
        if (opModeIsActive() && !isStopRequested()) {
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
                // Move to the left-side staging position with the intake stopped.
                Actions.runBlocking(new SequentialAction(
                        moveBack,
                        movetolaunchPositionLeft ));
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

                // Spin up for the second volley on the way to the launch location.
                launcher.setFlywheelVelocity(launcher.targetVelocity);
                Actions.runBlocking(launchPositionLeft);

                // Launch regardless of flywheel readiness, as with the first volley.
                if (opModeIsActive() && !isStopRequested()) {
                    Actions.runBlocking(new LauncherAction(launcher));
                }
                launcher.setFlywheelVelocity(0);

                // Park after the second launch.
                if (opModeIsActive() && !isStopRequested()) {
                    Actions.runBlocking(new SequentialAction(
                            park1,
                            park2 ));
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