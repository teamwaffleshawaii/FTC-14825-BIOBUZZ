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
                .splineTo(new Vector2d(25, -45), Math.toRadians(270))
                //...
                .build();

        Action flowerPosition2 = drive.actionBuilder(new Pose2d(25, -45, Math.toRadians(270) ))
                .strafeTo(new Vector2d(25, -50),
                        new TranslationalVelConstraint(30), new ProfileAccelConstraint(-10.0, 10.0)) //Do this path slower
                .build();

        Action movetolaunchPositionLeft = drive.actionBuilder(new Pose2d(25, -50, Math.toRadians(0) ))
                .strafeToLinearHeading(new Vector2d(-44,-26), Math.toRadians(90))
                .build();

        Action launchPositionLeft = drive.actionBuilder(new Pose2d(-44, -26, Math.toRadians(90) ))
                .strafeToLinearHeading(new Vector2d(-53,4), Math.toRadians(0))
                .build();

        //Create more trajectories as needed

        waitForStart();

        try {
        if (opModeIsActive() && !isStopRequested()) {
            // Spin up while driving to the first launch position.
            // Reaching target speed can take time; do not feed until ready.
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
                // Keep intake motors running while Road Runner follows BOTH paths.
                // The finally block stops the intake even if a trajectory throws.
                intakeSubsystem.intake(1);
                intakeSubsystem.rampDown();
                //turn on transfer motor too
                try {
                    Actions.runBlocking(new SequentialAction(
                            flowerPosition1,
                            flowerPosition1,
                            new SleepAction(2.0) // Keep collecting for 2 more seconds.
                    ));
                } finally {
                    intakeSubsystem.stopIntake();
                }
            }
        }
        } finally {
            launcher.setFlywheelVelocity(0);
            robot.shutdown();
        }
    }

}