
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
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherAction;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Sensors;

import java.util.Arrays;

@Autonomous(name = "Red Right", group = "Auto")
public class RedRight extends LinearOpMode {

    Robot robot;
    Actuators actuator;
    Sensors sensor;
    IntakeSubsystem intakeSubsystem;
    LauncherSubsystem launcher;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Robot(hardwareMap);

        actuator = robot.actuator;
        sensor = robot.sensor;
        intakeSubsystem = robot.intake;
        launcher = robot.launcher;

        // STARTING POSE
        Pose2d startPose = new Pose2d(
                60, -11.5, Math.toRadians(180)
        );

        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        // FAST CROSS-FIELD CONSTRAINTS
        VelConstraint fastVel = new MinVelConstraint(Arrays.asList(
                drive.kinematics.new WheelVelConstraint(95),
                new AngularVelConstraint(Math.PI * 1.5)
        ));

        AccelConstraint fastAccel =
                new ProfileAccelConstraint(-70, 75);

        // FIRST LAUNCH POSITION
        Action launchPositionRight =
                drive.actionBuilder(startPose)
                        .strafeToLinearHeading(
                                new Vector2d(47, -31),
                                Math.toRadians(160)
                        )
                        .build();

        // GROUND PICKUP
        Action pickupGroundPosition =
                drive.actionBuilder(
                                new Pose2d(47, -31, Math.toRadians(160))
                        )
                        .strafeToLinearHeading(
                                new Vector2d(50, -51),
                                Math.toRadians(0)
                        )
                        .strafeToLinearHeading(
                                new Vector2d(60, -54),
                                Math.toRadians(0)
                        )
                        .build();

        // RETURN TO LAUNCH POSITION
        Action launchPosition2 =
                drive.actionBuilder(
                                new Pose2d(60, -54, Math.toRadians(0))
                        )
                        .strafeToLinearHeading(
                                new Vector2d(52, -51),
                                Math.toRadians(0)
                        )
                        .strafeToLinearHeading(
                                new Vector2d(49, -31),
                                Math.toRadians(180)
                        )
                        .build();

        // FIRST FLOWER PICKUP
        Action flowerPosition =
                drive.actionBuilder(
                                new Pose2d(49, -31, Math.toRadians(180))
                        )
                        .splineTo(
                                new Vector2d(28, -42),
                                Math.toRadians(270)
                        )
                        .strafeTo(new Vector2d(23, -55))
                        .build();

        Action moveBack =
                drive.actionBuilder(
                                new Pose2d(23, -55, Math.toRadians(270))
                        )
                        .strafeToLinearHeading(
                                new Vector2d(28, -42),
                                Math.toRadians(270)
                        )
                        .build();

        // CROSS-FIELD PATH
        Action launchPositionLeft =
                drive.actionBuilder(
                                new Pose2d(28, -42, Math.toRadians(270))
                        )
                        .strafeToLinearHeading(
                                new Vector2d(0, -38),
                                Math.toRadians(0),
                                fastVel,
                                fastAccel
                        )
                        .strafeToLinearHeading(
                                new Vector2d(-45, -25),
                                Math.toRadians(20),
                                fastVel,
                                fastAccel
                        )
                        .build();

        // DIRECT CROSS-FIELD PATH IF NO TAG IS DETECTED
        Action launchPositionLeftFromGround =
                drive.actionBuilder(
                                new Pose2d(49, -25, Math.toRadians(180))
                        )
                        .strafeToLinearHeading(
                                new Vector2d(0, -33),
                                Math.toRadians(0),
                                fastVel,
                                fastAccel
                        )
                        .strafeToLinearHeading(
                                new Vector2d(-50, -23),
                                Math.toRadians(17),
                                fastVel,
                                fastAccel
                        )
                        .build();

        // LEFT-SIDE FLOWER PICKUP
        Action flowerPositionLeft =
                drive.actionBuilder(
                                new Pose2d(-45, -25, Math.toRadians(17))
                        )
                        .splineTo(
                                new Vector2d(-38, -30),
                                Math.toRadians(180)
                        )
                        .strafeTo(new Vector2d(-66, -10))
                        .build();

        Action moveBackLeft =
                drive.actionBuilder(
                                new Pose2d(-66, -10, Math.toRadians(180))
                        )
                        .strafeToLinearHeading(
                                new Vector2d(-40, -30),
                                Math.toRadians(180)
                        )
                        .build();

        // FINAL LEFT-SIDE LAUNCH PATH
        Action movetolaunchPositionAgainLeft =
                drive.actionBuilder(
                                new Pose2d(-40, -30, Math.toRadians(180))
                        )
                        .strafeToLinearHeading(
                                new Vector2d(-45, -15),
                                Math.toRadians(-1)
                        )
                        .strafeToLinearHeading(
                                new Vector2d(-45, -25),
                                Math.toRadians(17)
                        )
                        .build();

        // NORMAL PARK PATH
        Action park =
                drive.actionBuilder(
                                new Pose2d(-45, -25, Math.toRadians(17))
                        )
                        .strafeToLinearHeading(
                                new Vector2d(-30, -40),
                                Math.toRadians(90)
                        )
                        .strafeToLinearHeading(
                                new Vector2d(-25, -51),
                                Math.toRadians(90)
                        )
                        .build();

        // PARK DIRECTLY AFTER LEFT FLOWER PICKUP
        // Starts at the end of moveBackLeft.
        Action parkAfterLeftFlower =
                drive.actionBuilder(
                                new Pose2d(-40, -30, Math.toRadians(180))
                        )
                        .strafeToLinearHeading(
                                new Vector2d(-30, -40),
                                Math.toRadians(90)
                        )
                        .strafeToLinearHeading(
                                new Vector2d(-25, -55),
                                Math.toRadians(90)
                        )
                        .build();

        waitForStart();

        try {

            if (opModeIsActive() && !isStopRequested()) {

                // FIRST PRELOAD LAUNCH
                launcher.setFlywheelVelocity(
                        LauncherSubsystem.TARGET_VELOCITY
                );

                AutoLedAction.runBlocking(
                        actuator,
                        launcher,
                        launchPositionRight
                );

                if (opModeIsActive() && !isStopRequested()) {

                    AutoLedAction.runBlocking(
                            actuator,
                            launcher,
                            new AutoVisionLaunchAction(
                                    sensor, launcher, actuator, true
                            )
                    );

                    AutoLedAction.runBlocking(
                            actuator,
                            launcher,
                            new LauncherAction(launcher)
                    );
                }

                launcher.setFlywheelVelocity(0);

                // GROUND PICKUP
                if (opModeIsActive() && !isStopRequested()) {

                    intakeSubsystem.intake(1);

                    AutoLedAction.runBlocking(
                            actuator,
                            launcher,
                            new SequentialAction(
                                    pickupGroundPosition,
                                    new SleepAction(1.2),
                                    launchPosition2
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

                // CHECK FOR RED ALLIANCE APRILTAG
                boolean seesAllianceTag =
                        sensor.seesAllianceAprilTag(true, 0.1);

                if (seesAllianceTag) {

                    // LAUNCH GROUND PICKUP
                    if (opModeIsActive() && !isStopRequested()) {

                        launcher.setFlywheelVelocity(
                                LauncherSubsystem.TARGET_VELOCITY
                        );

                        AutoLedAction.runBlocking(
                                actuator,
                                launcher,
                                new AutoVisionLaunchAction(
                                        sensor, launcher, actuator, true
                                )
                        );

                        AutoLedAction.runBlocking(
                                actuator,
                                launcher,
                                new LauncherAction(launcher)
                        );

                        launcher.setFlywheelVelocity(0);
                    }

                    // FIRST FLOWER PICKUP
                    if (opModeIsActive() && !isStopRequested()) {

                        intakeSubsystem.rampDown();
                        intakeSubsystem.intake(1);

                        AutoLedAction.runBlocking(
                                actuator,
                                launcher,
                                new SequentialAction(
                                        flowerPosition,
                                        new SleepAction(1.2),
                                        moveBack
                                )
                        );
                    }

                    // CROSS TO LEFT SIDE
                    if (opModeIsActive() && !isStopRequested()) {

                        launcher.setFlywheelVelocity(
                                LauncherSubsystem.TARGET_VELOCITY
                        );

                        AutoLedAction.runBlocking(
                                actuator,
                                launcher,
                                launchPositionLeft
                        );
                    }

                } else {

                    // NO-TAG BRANCH:
                    // PRESERVE ORIGINAL LOGIC.
                    // SKIP GROUND LAUNCH AND FIRST FLOWER.

                    if (opModeIsActive() && !isStopRequested()) {

                        launcher.setFlywheelVelocity(
                                LauncherSubsystem.TARGET_VELOCITY
                        );

                        AutoLedAction.runBlocking(
                                actuator,
                                launcher,
                                launchPositionLeftFromGround
                        );
                    }
                }

                // LEFT-SIDE LAUNCH
                // RUNS FOR BOTH TAG AND NO-TAG BRANCHES.
                if (opModeIsActive() && !isStopRequested()) {

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

                    AutoLedAction.runBlocking(
                            actuator,
                            launcher,
                            new AutoVisionLaunchAction(
                                    sensor, launcher, actuator, true
                            )
                    );

                    AutoLedAction.runBlocking(
                            actuator,
                            launcher,
                            new LauncherAction(launcher)
                    );

                    launcher.setFlywheelVelocity(0);
                }

                // LEFT-SIDE FLOWER PICKUP
                if (opModeIsActive() && !isStopRequested()) {

                    intakeSubsystem.rampDown();
                    intakeSubsystem.intake(1);

                    AutoLedAction.runBlocking(
                            actuator,
                            launcher,
                            new SequentialAction(
                                    flowerPositionLeft,
                                    new SleepAction(1.2),
                                    moveBackLeft
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

                // FINAL LEFT-SIDE LAUNCH
                // ONLY RUN IF NO APRILTAG WAS DETECTED.
                if (!seesAllianceTag
                        && opModeIsActive()
                        && !isStopRequested()) {

                    launcher.setFlywheelVelocity(
                            LauncherSubsystem.TARGET_VELOCITY
                    );

                    AutoLedAction.runBlocking(
                            actuator,
                            launcher,
                            movetolaunchPositionAgainLeft
                    );

                    launcher.requireBallBeforeKicker = true;

                    AutoLedAction.runBlocking(
                            actuator,
                            launcher,
                            new AutoVisionLaunchAction(
                                    sensor, launcher, actuator, true
                            )
                    );

                    AutoLedAction.runBlocking(
                            actuator,
                            launcher,
                            new LauncherAction(launcher)
                    );

                    launcher.setFlywheelVelocity(0);
                }

                // PARK
                if (opModeIsActive() && !isStopRequested()) {

                    if (seesAllianceTag) {

                        // TAG DETECTED:
                        // SKIP FINAL LAUNCH AND PARK AFTER FLOWER PICKUP.
                        launcher.setFlywheelVelocity(0);

                        AutoLedAction.runBlocking(
                                actuator,
                                launcher,
                                parkAfterLeftFlower
                        );

                    } else {

                        // NO TAG:
                        // KEEP ORIGINAL FINAL LAUNCH AND NORMAL PARK.
                        AutoLedAction.runBlocking(
                                actuator,
                                launcher,
                                park
                        );
                    }
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
