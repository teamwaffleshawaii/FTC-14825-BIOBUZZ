//Auto Template that you can use to program your auto
// NOTE: THIS CODE RUNS ON ODOMETRY, you must have your odometry tuned

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
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
            //Run your trajectory here when the code starts
            Actions.runBlocking(new SequentialAction(launchPositionRight));
            //Turn the launch motor on
            //Launch the 4 balls using the Finite State Machine
            //Turn the launch motor off
            //Intake on
            Actions.runBlocking(new SequentialAction(intakePosition1, intakePosition2));
            //Intake off
        }
        robot.shutdown();
    }

}