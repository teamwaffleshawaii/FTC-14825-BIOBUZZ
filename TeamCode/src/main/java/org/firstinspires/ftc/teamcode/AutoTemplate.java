//Auto Template that you can use to program your auto
// NOTE: THIS CODE RUNS ON ODOMETRY, you must have your odometry tuned

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.odometry.MecanumDrive;

@Autonomous
public class AutoTemplate extends LinearOpMode {

    //Include all of your actuators
    Actuators actuator = new Actuators();

    @Override
    public void runOpMode() throws InterruptedException {
        //Include all of your motors hardware
        actuator.init(hardwareMap);

        //TODO: Set up Robot starting position here!!!
        MecanumDrive drive = new MecanumDrive(hardwareMap,
                new Pose2d(0, 0, Math.toRadians(90) ) );

        //TODO: Put your Odometry trajectory paths here!!!
        //Separate your paths by action (linearSlide movements, intake movements, servo movements, etc.)
        //Work shop it line 35,36
        Action trajectory1 = drive.actionBuilder(new Pose2d(0, 0, Math.toRadians(90) ))
                .strafeToLinearHeading(new Vector2d( -42, 32), Math.toRadians(-52))
                .strafeToLinearHeading(new Vector2d( 34, 1), Math.toRadians(93))
                .strafeToLinearHeading(new Vector2d( 0, 0), Math.toRadians(90))
                //...
                .build();

	    //TODO: Your trajectory 2 starting pose must match where the end of your trajectory 1 should be
        Action trajectory2 = drive.actionBuilder(new Pose2d(-34, -29, Math.toRadians(127) ))
                .strafeToLinearHeading(new Vector2d(-23, -23), Math.toRadians(90))
                //...
                .build();

	    //TODO: Your trajectory 3 starting pose must match where the end of your trajectory 2 should be
        Action trajectory3 = drive.actionBuilder(new Pose2d(0, 0, Math.toRadians(90) ))
                .strafeToLinearHeading(new Vector2d(-40,-30), Math.toRadians(-180))
                .build();

        Action trajectory4 = drive.actionBuilder(new Pose2d(-40, -30, Math.toRadians(-180) ))
                .strafeToLinearHeading(new Vector2d(0,0), Math.toRadians(90))
                .build();
        //Create more trajectories as needed

        while (!isStarted() && !isStopRequested()) {
            //include all of your hardware configurations
            actuator.init(hardwareMap);
        }

        waitForStart();

        if (opModeIsActive() && !isStopRequested()) {
            //Run your trajectory here when the code starts
          Actions.runBlocking(new SequentialAction(trajectory1));
            //actuator.setServoPosition(0.44); //close servo
            //...
            //...
            //...
        }
    }

}