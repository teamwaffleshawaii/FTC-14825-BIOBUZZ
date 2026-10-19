//Auto Template that you can use to program your auto
// NOTE: THIS CODE RUNS ON ODOMETRY, you must have your odometry tuned

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
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

        Action intakePosition1 = drive.actionBuilder(new Pose2d(47, -11.5, Math.toRadians(180) ))
                .strafeToLinearHeading(new Vector2d(63, -33), Math.toRadians(270))
                //...
                .build();

        Action intakePosition2 = drive.actionBuilder(new Pose2d(63, -33, Math.toRadians(270) ))
                .strafeToLinearHeading(new Vector2d(63, -56), Math.toRadians(270),
                        new TranslationalVelConstraint(30), new ProfileAccelConstraint(-10.0, 10.0)) //Do this path slower
                .build();

        Action launchPositionLeft = drive.actionBuilder(new Pose2d(63, -56, Math.toRadians(270) ))
                .strafeToLinearHeading(new Vector2d(0,0), Math.toRadians(90))
                .build();

        //Create more trajectories as needed

        waitForStart();

        if (opModeIsActive() && !isStopRequested()) {
            // Spin up while driving to the first launch position.
            // Reaching target speed can take time; do not feed until ready.
            launcher.setFlywheelVelocity(launcher.targetVelocity);
            Actions.runBlocking(launchPositionRight);

            // Wait up to 3 seconds for the flywheel to reach its target speed.
            // An unsafe/failed spin-up will skip the shot rather than feed slowly.
            com.qualcomm.robotcore.util.ElapsedTime readyTimer =
                    new com.qualcomm.robotcore.util.ElapsedTime();
            while (opModeIsActive() && !isStopRequested()
                    && !launcher.isFlywheelReady() && readyTimer.seconds() < 3.0) {
                telemetry.addData("Launch velocity", actuator.getLaunchVelocity());
                telemetry.addData("Launcher ready", false);
                telemetry.update();
                idle();
            }

            if (opModeIsActive() && !isStopRequested() && launcher.isFlywheelReady()) {
                // Reuse TeleOp's non-blocking four-ball finite state machine.
                Actions.runBlocking(new LauncherAction(launcher));
            } else {
                telemetry.addLine("Launcher not ready: skipping four-ball launch");
                telemetry.update();
            }

            // Stop the flywheel regardless of whether the first shot succeeded.
            launcher.setFlywheelVelocity(0);

            if (opModeIsActive() && !isStopRequested()) {
                // Keep intake motors running while Road Runner follows BOTH paths.
                Actions.runBlocking(new SequentialAction(
                        new InstantAction(() -> intakeSubsystem.intake(1)),
                        intakePosition1,
                        intakePosition2,
                        new InstantAction(() -> intakeSubsystem.stopIntake())
                ));
            }
        }
        robot.shutdown();
    }

}