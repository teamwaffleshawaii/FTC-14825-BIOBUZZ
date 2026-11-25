
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

@Autonomous(name = "Blue Right", group = "Auto")
public class BlueRight extends LinearOpMode {

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

        Pose2d startPose = new Pose2d(
                -60, 11.5, Math.toRadians(0)
        );

        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        // Faster constraints for the long cross-field movement.
        VelConstraint fastVel = new MinVelConstraint(Arrays.asList(
                drive.kinematics.new WheelVelConstraint(95),
                new AngularVelConstraint(Math.PI * 1.5)
        ));

        AccelConstraint fastAccel =
                new ProfileAccelConstraint(-70, 75);

        // FIRST LAUNCH POSITION
        Action launchPositionRight = drive.actionBuilder(startPose)
                .splineToLinearHeading(
                        new Pose2d(-40, 29, Math.toRadians(-20)),
                        Math.toRadians(40)
                )
                .build();

        // GROUND PICKUP
        Action pickupGroundPosition =
                drive.actionBuilder(
                                new Pose2d(-40, 29, Math.toRadians(-20))
                        )
                        .splineToLinearHeading(
                                new Pose2d(-50, 51, Math.toRadians(180)),
                                Math.toRadians(135)
                        )
                        .strafeTo(new Vector2d(-62, 51))
                        .build();

        // RETURN TO FIRST LAUNCH POSITION
        Action launchPosition2 =
                drive.actionBuilder(
                                new Pose2d(-62, 51, Math.toRadians(180))
                        )
                        .strafeTo(new Vector2d(-45, 51))
                        .splineToLinearHeading(
                                new Pose2d(-49, 30, Math.toRadians(-20)),
                                Math.toRadians(-90)
                        )
                        .build();

        // FIRST FLOWER PICKUP
        Action flowerPosition =
                drive.actionBuilder(
                                new Pose2d(-49, 30, Math.toRadians(-20))
                        )
                        .splineTo(
                                new Vector2d(-28, 48),
                                Math.toRadians(90)
                        )
                        .strafeTo(new Vector2d(-20, 61))
                        .build();

        Action moveBack =
                drive.actionBuilder(
                                new Pose2d(-20, 61, Math.toRadians(90))
                        )
                        .strafeToLinearHeading(
                                new Vector2d(-28, 61),
                                Math.toRadians(90)
                        )
                        .build();

        // CROSS THE FIELD TO SECOND LAUNCH POSITION
        Action launchPositionLeft =
                drive.actionBuilder(
                                new Pose2d(-28, 61, Math.toRadians(90))
                        )
                        .strafeToLinearHeading(
                                new Vector2d(0, 38),
                                Math.toRadians(180),
                                fastVel,
                                fastAccel
                        )
                        .splineToLinearHeading(
                                new Pose2d(38, 20, Math.toRadians(200)),
                                Math.toRadians(-10),
                                fastVel,
                                fastAccel
                        )
                        .build();

        // Direct crossing if the first ground pickup is skipped.
        Action launchPositionLeftFromGround =
                drive.actionBuilder(
                                new Pose2d(-49, 20, Math.toRadians(-20))
                        )
                        .strafeToLinearHeading(
                                new Vector2d(0, 38),
                                Math.toRadians(180),
                                fastVel,
                                fastAccel
                        )
                        .splineToLinearHeading(
                                new Pose2d(38, 20, Math.toRadians(200)),
                                Math.toRadians(-10),
                                fastVel,
                                fastAccel
                        )
                        .build();

        // LEFT FLOWER PICKUP
        Action flowerPositionLeft =
                drive.actionBuilder(
                                new Pose2d(38, 20, Math.toRadians(200))
                        )
                        .splineTo(
                                new Vector2d(38, 36),
                                Math.toRadians(0)
                        )
                        .strafeTo(new Vector2d(61, 23))
                        .build();

        // Back away from the left flower.
        Action moveBackLeft =
                drive.actionBuilder(
                                new Pose2d(61, 23, Math.toRadians(0))
                        )
                        .splineToLinearHeading(
                                new Pose2d(40, 36, Math.toRadians(0)),
                                Math.toRadians(180)
                        )
                        .build();

        // RETURN TO LEFT LAUNCH POSITION
        Action movetolaunchPositionAgainLeft =
                drive.actionBuilder(
                                new Pose2d(40, 36, Math.toRadians(0))
                        )
                        .splineToLinearHeading(
                                new Pose2d(47, 20, Math.toRadians(179)),
                                Math.toRadians(-90)
                        )
                        .splineToLinearHeading(
                                new Pose2d(38, 23, Math.toRadians(200)),
                                Math.toRadians(90)
                        )
                        .build();

        // NORMAL PARK
        Action park =
                drive.actionBuilder(
                                new Pose2d(38, 23, Math.toRadians(200))
                        )
                        .splineToLinearHeading(
                                new Pose2d(50, 50, Math.toRadians(270)),
                                Math.toRadians(90)
                        )
                        .build();

        // PARK AFTER LEFT FLOWER
        // Starts at the end of moveBackLeft.
        Action parkAfterLeftFlower =
                drive.actionBuilder(
                                new Pose2d(40, 36, Math.toRadians(0))
                        )
                        .splineToLinearHeading(
                                new Pose2d(55, 48, Math.toRadians(270)),
                                Math.toRadians(270)
                        )
                        .build();

        // AUTONOMOUS SEQUENCE
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
                                    sensor, launcher, actuator, false
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

                // CHECK FOR BLUE ALLIANCE APRILTAG
                boolean seesAllianceTag =
                        sensor.seesAllianceAprilTag(false, 0.5);

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

                    // FIRST FLOWER
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

                    // NO-TAG BRANCH: ORIGINAL LOGIC PRESERVED.
                    // SKIP GROUND LAUNCH AND FIRST FLOWER.
                    // DIRECT CROSS TO LEFT SIDE.

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

                // LEFT FLOWER PICKUP
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

                // PARK
                if (opModeIsActive() && !isStopRequested()) {

                    if (seesAllianceTag) {

                        // Tag detected: skip final launch and park
                        // from the left flower pickup position.
                        launcher.setFlywheelVelocity(0);

                        AutoLedAction.runBlocking(
                                actuator,
                                launcher,
                                parkAfterLeftFlower
                        );

                    } else {

                        // No tag: original final launch and parking route.
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
