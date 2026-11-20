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

import java.util.Arrays;

/**
 * Autonomous route starting one tile further down from Red Right (Y = -36).
 * Moves forward toward goal, launches preloads, drives straight across to the
 * other side of the hive, picks up from the flower, turns around and launches.
 */
@Autonomous (name = "RedOffset", group = "Auto")
public class RedOffset extends LinearOpMode {

    // References to shared Robot subsystems.
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

        // Starting pose: 1 tile further down in Y from Red Right (60, -36, 180°).
        MecanumDrive drive = new MecanumDrive(hardwareMap,
                new Pose2d(60, -36, Math.toRadians(180)));

        // Fast crossing constraints across the field
        VelConstraint fastVel = new MinVelConstraint(Arrays.asList(
                drive.kinematics.new WheelVelConstraint(95),
                new AngularVelConstraint(Math.PI * 1.5)
        ));
        AccelConstraint fastAccel = new ProfileAccelConstraint(-75, 90);

        // 1. Drive forward into first launch position
        Action driveToLaunchPosition = drive.actionBuilder(new Pose2d(60, -36, Math.toRadians(180)))
                .strafeToLinearHeading(new Vector2d(45, -36), Math.toRadians(160))
                .build();

        // 2. Drive straight across to the other side of the hive, angled slightly left toward the flower
        Action driveToOtherSide = drive.actionBuilder(new Pose2d(45, -36, Math.toRadians(160)))
                .splineToConstantHeading(new Vector2d(-34, -50), Math.toRadians(180), fastVel, fastAccel)
                .build();

        // 3. Flower pickup on the other side
        Action flowerPosition = drive.actionBuilder(new Pose2d(-40, -45, Math.toRadians(180)))
                .strafeTo(new Vector2d(-63, -26))
                .build();

        Action moveBack = drive.actionBuilder(new Pose2d(-63, -26, Math.toRadians(180)))
                .strafeToLinearHeading(new Vector2d(-40, -30), Math.toRadians(180))
                .build();

        // 4. Turn around to face goal for launching flower balls
        Action turnAroundToLaunch = drive.actionBuilder(new Pose2d(-40, -30, Math.toRadians(180)))
                .strafeToLinearHeading(new Vector2d(-45, -10), Math.toRadians(-1))
                .strafeToLinearHeading(new Vector2d(-45, -30), Math.toRadians(20))
                .build();

        // 5. Park after launching
        Action park = drive.actionBuilder(new Pose2d(-45, -30, Math.toRadians(20)))
                .strafeToLinearHeading(new Vector2d(-50, -40), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-50, -61), Math.toRadians(90))
                .build();

        waitForStart();

        try {
            if (opModeIsActive() && !isStopRequested()) {
                // Spin up flywheel while driving to launch position
                launcher.setFlywheelVelocity(LauncherSubsystem.TARGET_VELOCITY);

                AutoLedAction.runBlocking(actuator, launcher, new SequentialAction(
                        driveToLaunchPosition
                ));

                // Aim turret with Limelight vision, lock speed, and launch preloads
                if (opModeIsActive() && !isStopRequested()) {
                    AutoLedAction.runBlocking(actuator, launcher, new AutoVisionLaunchAction(sensor, launcher, actuator, true));
                    AutoLedAction.runBlocking(actuator, launcher, new LauncherAction(launcher));
                }
                launcher.setFlywheelVelocity(0);

                // Drive straight across to the other side of the hive
                if (opModeIsActive() && !isStopRequested()) {
                    AutoLedAction.runBlocking(actuator, launcher, driveToOtherSide);
                }

                // Pick up from the flower on the other side
                if (opModeIsActive() && !isStopRequested()) {
                    intakeSubsystem.rampDown();
                    intakeSubsystem.intake(1);
                    AutoLedAction.runBlocking(actuator, launcher, new SequentialAction(
                            flowerPosition,
                            new SleepAction(1.2),
                            moveBack
                    ));

                    // After backing out of flower, raise ramp, stop intake, and reverse transfer
                    intakeSubsystem.rampUp();
                    intakeSubsystem.stopIntake();
                    intakeSubsystem.startStepUpReversePulse();
                    AutoLedAction.runBlocking(actuator, launcher, (packet) -> {
                        intakeSubsystem.update();
                        return intakeSubsystem.isStepUpReverseActive();
                    });
                    intakeSubsystem.stopFeeding();
                }

                // Turn around to launch position and launch flower balls
                if (opModeIsActive() && !isStopRequested()) {
                    launcher.setFlywheelVelocity(LauncherSubsystem.TARGET_VELOCITY);
                    AutoLedAction.runBlocking(actuator, launcher, turnAroundToLaunch);

                    launcher.requireBallBeforeKicker = true;

                    if (opModeIsActive() && !isStopRequested()) {
                        AutoLedAction.runBlocking(actuator, launcher, new AutoVisionLaunchAction(sensor, launcher, actuator, true));
                        AutoLedAction.runBlocking(actuator, launcher, new LauncherAction(launcher));
                    }
                    launcher.setFlywheelVelocity(0);
                }

                // Park after launching
                if (opModeIsActive() && !isStopRequested()) {
                    AutoLedAction.runBlocking(actuator, launcher, new SequentialAction(
                            park
                    ));
                }
            }
        } finally {
            launcher.setFlywheelVelocity(0);
            intakeSubsystem.rampUp();
            robot.shutdown();
            sleep(1000);
        }
    }
}
