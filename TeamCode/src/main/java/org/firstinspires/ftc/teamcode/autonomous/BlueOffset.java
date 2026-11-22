
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

@Autonomous(name = "BlueOffset", group = "Auto")
public class BlueOffset extends LinearOpMode {

    Robot robot;
    Actuators actuator;
    Sensors sensor;
    IntakeSubsystem intakeSubsystem;
    LauncherSubsystem launcher;

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize robot and subsystems.
        robot = new Robot(hardwareMap);

        actuator = robot.actuator;
        sensor = robot.sensor;
        intakeSubsystem = robot.intake;
        launcher = robot.launcher;

        // Starting pose.
        Pose2d startPose = new Pose2d(
                -60, 36, Math.toRadians(0)
        );

        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        // Fast crossing constraints.
        VelConstraint fastVel = new MinVelConstraint(Arrays.asList(
                drive.kinematics.new WheelVelConstraint(95),
                new AngularVelConstraint(Math.PI * 1.5)
        ));

        AccelConstraint fastAccel =
                new ProfileAccelConstraint(-75, 90);

        // 1. Move to the original-side launch position.
        Action driveToLaunchPosition =
                drive.actionBuilder(startPose)
                        .strafeToLinearHeading(
                                new Vector2d(-48, 36),
                                Math.toRadians(330)
                        )
                        .build();

        // 2. Ground pickup.
        Action pickupGroundPosition =
                drive.actionBuilder(
                                new Pose2d(-48, 36, Math.toRadians(330))
                        )
                        .strafeToLinearHeading(
                                new Vector2d(-50, 51),
                                Math.toRadians(180)
                        )
                        .strafeToLinearHeading(
                                new Vector2d(-60, 54),
                                Math.toRadians(180)
                        )
                        .build();

        // 3. Cross directly from ground pickup to the opposite side.
        Action crossToOppositeLaunch =
                drive.actionBuilder(
                                new Pose2d(-60, 54, Math.toRadians(180))
                        )
                        .strafeToLinearHeading(
                                new Vector2d(-40, 33),
                                Math.toRadians(180),
                                fastVel,
                                fastAccel
                        )
                        .splineToConstantHeading(
                                new Vector2d(46, 21),
                                Math.toRadians(187),
                                fastVel,
                                fastAccel
                        )
                        .build();

        // 4. Approach the opposite-side flower.
        Action secondFlowerApproach =
                drive.actionBuilder(
                                new Pose2d(46, 21, Math.toRadians(200))
                        )
                        .strafeToLinearHeading(
                                new Vector2d(45, 10),
                                Math.toRadians(179),
                                fastVel,
                                fastAccel
                        )
                        .strafeToLinearHeading(
                                new Vector2d(40, 30),
                                Math.toRadians(0),
                                fastVel,
                                fastAccel
                        )
                        .build();

        Action secondFlowerPosition =
                drive.actionBuilder(
                                new Pose2d(40, 30, Math.toRadians(0))
                        )
                        .strafeToConstantHeading(
                                new Vector2d(50, 36)
                        )
                        .strafeTo(
                                new Vector2d(65, 20)
                        )
                        .build();

        Action secondFlowerMoveBack =
                drive.actionBuilder(
                                new Pose2d(65, 20, Math.toRadians(0))
                        )
                        .strafeToLinearHeading(
                                new Vector2d(40, 36),
                                Math.toRadians(0)
                        )
                        .build();

        // 5. Return to the opposite-side launch position.
        Action returnToOppositeLaunch =
                drive.actionBuilder(
                                new Pose2d(40, 36, Math.toRadians(0))
                        )
                        .strafeToLinearHeading(
                                new Vector2d(45, 10),
                                Math.toRadians(179)
                        )
                        .strafeToLinearHeading(
                                new Vector2d(55, 25),
                                Math.toRadians(197)
                        )
                        .build();

        // 6. Park from the opposite side.
        Action park =
                drive.actionBuilder(
                                new Pose2d(55, 25, Math.toRadians(197))
                        )
                        .strafeToLinearHeading(
                                new Vector2d(30, 40),
                                Math.toRadians(270)
                        )
                        .strafeToLinearHeading(
                                new Vector2d(25, 58),
                                Math.toRadians(270)
                        )
                        .build();

        waitForStart();

        try {

            if (opModeIsActive() && !isStopRequested()) {

                // FIRST LAUNCH: PRELOAD BALLS.
                launcher.setFlywheelVelocity(
                        LauncherSubsystem.TARGET_VELOCITY
                );

                AutoLedAction.runBlocking(
                        actuator,
                        launcher,
                        driveToLaunchPosition
                );

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

                launcher.setFlywheelVelocity(0);

                // GROUND PICKUP.
                if (opModeIsActive() && !isStopRequested()) {

                    intakeSubsystem.intake(1);

                    AutoLedAction.runBlocking(
                            actuator,
                            launcher,
                            new SequentialAction(
                                    pickupGroundPosition,
                                    new SleepAction(1.2)
                            )
                    );

                    intakeSubsystem.stopIntake();

                    intakeSubsystem.startStepUpReversePulse();

                    AutoLedAction.runBlocking(
                            actuator,
                            launcher,
                            packet -> {
                                intakeSubsystem.update();
                                return intakeSubsystem.isStepUpReverseActive();
                            }
                    );

                    intakeSubsystem.stopFeeding();
                }

                // CROSS DIRECTLY TO THE OPPOSITE SIDE.
                if (opModeIsActive() && !isStopRequested()) {

                    launcher.setFlywheelVelocity(
                            LauncherSubsystem.TARGET_VELOCITY
                    );

                    AutoLedAction.runBlocking(
                            actuator,
                            launcher,
                            crossToOppositeLaunch
                    );
                }

                // SECOND LAUNCH: GROUND PICKUP.
                if (opModeIsActive() && !isStopRequested()) {

                    launcher.requireBallBeforeKicker = true;

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

                    launcher.setFlywheelVelocity(0);
                }

                // OPPOSITE-SIDE FLOWER PICKUP.
                if (opModeIsActive() && !isStopRequested()) {

                    intakeSubsystem.rampDown();
                    intakeSubsystem.intake(1);

                    AutoLedAction.runBlocking(
                            actuator,
                            launcher,
                            new SequentialAction(
                                    secondFlowerApproach,
                                    secondFlowerPosition,
                                    new SleepAction(2.2),
                                    secondFlowerMoveBack
                            )
                    );

                    intakeSubsystem.rampUp();
                    intakeSubsystem.stopIntake();

                    intakeSubsystem.startStepUpReversePulse();

                    AutoLedAction.runBlocking(
                            actuator,
                            launcher,
                            packet -> {
                                intakeSubsystem.update();
                                return intakeSubsystem.isStepUpReverseActive();
                            }
                    );

                    intakeSubsystem.stopFeeding();
                }

                // RETURN TO THE OPPOSITE-SIDE LAUNCH POSITION.
                if (opModeIsActive() && !isStopRequested()) {

                    launcher.setFlywheelVelocity(
                            LauncherSubsystem.TARGET_VELOCITY
                    );

                    AutoLedAction.runBlocking(
                            actuator,
                            launcher,
                            returnToOppositeLaunch
                    );
                }

                // THIRD LAUNCH: SECOND FLOWER.
                if (opModeIsActive() && !isStopRequested()) {

                    launcher.requireBallBeforeKicker = true;

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

                    launcher.setFlywheelVelocity(0);
                }

                // PARK.
                if (opModeIsActive() && !isStopRequested()) {

                    AutoLedAction.runBlocking(
                            actuator,
                            launcher,
                            park
                    );
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
