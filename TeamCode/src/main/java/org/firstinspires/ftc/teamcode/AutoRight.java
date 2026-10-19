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
        MecanumDrive drive = new MecanumDrive(hardwareMap,
                new Pose2d(60, -11.5, Math.toRadians(180) ) );

        //TODO: Put your Odometry trajectory paths here!!!
        //Separate your paths by action (linearSlide movements, intake movements, servo movements, etc.)
        Action launchPositionRight = drive.actionBuilder(new Pose2d(60, -11.5, Math.toRadians(180) ))
                .strafeToLinearHeading(new Vector2d( 47, -11.5), Math.toRadians(180))
                .build();

        Action flowerPosition1 = drive.actionBuilder(new Pose2d(47, -11.5, Math.toRadians(180) ))
                .splineTo(new Vector2d(30, -44), Math.toRadians(270))
                //...
                .build();

        Action flowerPosition2 = drive.actionBuilder(new Pose2d(30, -44, Math.toRadians(270) ))
                .strafeTo(new Vector2d(25, -55))
                .build();

        // Small odometry-controlled forward/backward wiggle at the flowers.
        // Heading is 270 degrees, so moving to y=-56 advances one inch,
        // then returning to y=-55 backs up one inch.
        Action flowerWiggle = drive.actionBuilder(new Pose2d(25, -55, Math.toRadians(270)))
                .strafeTo(new Vector2d(25, -56),
                        new TranslationalVelConstraint(8),
                        new ProfileAccelConstraint(-8.0, 8.0))
                .strafeTo(new Vector2d(25, -55),
                        new TranslationalVelConstraint(8),
                        new ProfileAccelConstraint(-8.0, 8.0))
                .strafeTo(new Vector2d(25, -56),
                        new TranslationalVelConstraint(8),
                        new ProfileAccelConstraint(-8.0, 8.0))
                .strafeTo(new Vector2d(25, -55),
                        new TranslationalVelConstraint(8),
                        new ProfileAccelConstraint(-8.0, 8.0))
                .build();

        Action movetolaunchPositionLeft = drive.actionBuilder(new Pose2d(25, -55, Math.toRadians(270) ))
                .strafeToLinearHeading(new Vector2d(-44,-35), Math.toRadians(90))
                .build();

        Action launchPositionLeft = drive.actionBuilder(new Pose2d(-47, -31, Math.toRadians(85) ))
                .strafeToLinearHeading(new Vector2d(-54,-1.5), Math.toRadians(-5))
                .build();

        Action park = drive.actionBuilder(new Pose2d(-54, -1.5, Math.toRadians(-5) ))
                .strafeToLinearHeading(new Vector2d(-54,-44), Math.toRadians(90))
                .build();

        //Create more trajectories as needed

        waitForStart();

        try {
        if (opModeIsActive() && !isStopRequested()) {
            // Spin up while driving to the first launch position.
            // The first volley starts after arriving, regardless of actual speed.
            launcher.setFlywheelVelocity(launcher.targetVelocity);
            Actions.runBlocking(launchPositionRight);

            // Launch regardless of flywheel readiness so the preload balls
            // are cleared before collecting the next four.
            if (opModeIsActive() && !isStopRequested()) {
                Actions.runBlocking(new LauncherAction(launcher));
            }

            // Stop the flywheel regardless of whether the first shot succeeded.
            launcher.setFlywheelVelocity(0);

            if (opModeIsActive() && !isStopRequested()) {
                // Both intake and transfer run continuously while the robot wiggles.
                intakeSubsystem.rampDown();
                intakeSubsystem.intake(1); // Intake and transfer stay on continuously.
                try {
                    Actions.runBlocking(new SequentialAction(
                            flowerPosition1,
                            flowerPosition2,
                            flowerWiggle,
                            new SleepAction(3.5)
                    ));
                } finally {
                    intakeSubsystem.stopIntake();
                }
            }

            if (opModeIsActive() && !isStopRequested()) {
                // Move to the left-side staging position with the intake stopped.
                Actions.runBlocking(movetolaunchPositionLeft);
                intakeSubsystem.rampUp();

                // Spin up for the second volley on the way to the launch location.
                launcher.setFlywheelVelocity(launcher.targetVelocity);
                Actions.runBlocking(launchPositionLeft);

                // Launch regardless of flywheel readiness, as with the first volley.
                if (opModeIsActive() && !isStopRequested()) {
                    Actions.runBlocking(new LauncherAction(launcher));
                }
                launcher.setFlywheelVelocity(0);
            }
        }
        } finally {
            launcher.setFlywheelVelocity(0);
            intakeSubsystem.rampUp();
            robot.shutdown();
        }
    }

}