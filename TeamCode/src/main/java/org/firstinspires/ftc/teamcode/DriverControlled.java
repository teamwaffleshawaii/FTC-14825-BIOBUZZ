package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class DriverControlled extends LinearOpMode {

    double reset_angle = 0;
    Actuators actuator = new Actuators();
    Sensors sensor = new Sensors();

    //Set the launch motor velocity
    double targetVelocity = 1060;
    //Read the actual velocity
    double currentVelocity = 0;
    //Check whether launch is ready
    boolean launcherReady = false;

    //Cases for finite state machine for kicker
    public enum KickerState {
        KICKER_START,
        KICKER_RESET,
        KICKER_UP
    }

    //kickerState variable is declared
    KickerState kickerState = KickerState.KICKER_START;
    //timer for kicker FSM
    public ElapsedTime kickerTimer = new ElapsedTime();

    @Override
    public void runOpMode() {

        actuator.init(hardwareMap);
        sensor.init(hardwareMap);

        //reset FSM timer
        kickerTimer.reset();

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                drive();
                resetAngle();
                intake();
                flowerRamp();
                launch();
                kicker(); //kicker is now a FSM
                limelightTelemetry();
                telemetry.update();
            }
        }
        sensor.stopLimelight();
    }

    public void drive() {
        double Protate = - gamepad1.right_stick_x;
        double stick_x = - gamepad1.left_stick_x * Math.sqrt(Math.pow(1-Math.abs(Protate), 2) );
        //Accounts for Protate when limiting magnitude to be less than 1
        double stick_y = - gamepad1.left_stick_y * Math.sqrt(Math.pow(1-Math.abs(Protate), 2) );
        double theta = 0;
        double Px = 0;
        double Py = 0;

        double gyroAngle = getHeading() * Math.PI / 180; //Converts gyroAngle into radians
        if (gyroAngle <= 0) {
            gyroAngle = gyroAngle + (Math.PI / 2);
        } else if (0 < gyroAngle && gyroAngle < Math.PI / 2) {
            gyroAngle = gyroAngle + (Math.PI / 2);
        } else if (Math.PI / 2 <= gyroAngle) {
            gyroAngle = gyroAngle - (3 * Math.PI / 2);
        }
        gyroAngle = -1 * gyroAngle;

        if(gamepad1.right_bumper){ //Disables gyro, sets to -Math.PI/2 so front is defined correctly.
            gyroAngle = -Math.PI/2;
        }

        //Linear directions in case you want to do straight lines.
        if(gamepad1.dpad_right){
            stick_x = 0.5;
        }
        else if(gamepad1.dpad_left){
            stick_x = -0.5;
        }
        if(gamepad1.dpad_up){
            stick_y = -0.5;
        }
        else if(gamepad1.dpad_down){
            stick_y = 0.5;
        }

        //MOVEMENT
        theta = Math.atan2(stick_y, stick_x) - gyroAngle - (Math.PI / 2);
        Px = Math.sqrt(Math.pow(stick_x, 2) + Math.pow(stick_y, 2)) * (Math.sin(theta + Math.PI / 4));
        Py = Math.sqrt(Math.pow(stick_x, 2) + Math.pow(stick_y, 2)) * (Math.sin(theta - Math.PI / 4));

        actuator.leftFront((Py - Protate));
        actuator.leftBack((Px - Protate));
        actuator.rightBack((Py + Protate));
        actuator.rightFront((Px + Protate));
    }
    public void resetAngle(){
        if(gamepad1.start){
            reset_angle = getHeading() + reset_angle;
        }
    }
    public double getHeading(){

        double heading;
        //Get heading from the IMU or the navX
        heading = sensor.getHeading(AngleUnit.DEGREES); //Detecting yaw value from imu

        if(heading < -180) {
            heading = heading + 360;
        }

        else if(heading > 180) {
            heading = heading - 360;
        }

        heading = heading - reset_angle;
        telemetry.addData("Heading (Degrees)", heading);
        telemetry.addData("Heading (Radians)", heading * 3.1415926535 / 90);
        return heading;
    }
    public void flowerRamp(){
        if (gamepad1.left_trigger_pressed || gamepad1.left_bumper){
            actuator.intakeRampDown();
        } else {
            actuator.intakeRampUp();
        }
    }
    public void intake() {
        if (gamepad1.a || gamepad2.a) {
            actuator.intakeMotor.setPower(1);
            actuator.transferMotor.setPower(1);
        } else if (gamepad1.b || gamepad2.b) {
            actuator.intakeMotor.setPower(-0.8);
            actuator.transferMotor.setPower(-0.8);
        } else if (gamepad2.right_bumper) {
            actuator.stepUpLeft.setPower(1);
            actuator.stepUpRight.setPower(1);
            actuator.intakeMotor.setPower(0.5);
            actuator.transferMotor.setPower(1);
        } else {
            actuator.intakeMotor.setPower(0);
            actuator.transferMotor.setPower(0);
        }
    }
    public void launch() {
        if (gamepad2.x) {
            actuator.launchMotor(targetVelocity); //adjust this as needed
            //later on, this should probably be auto-adjusted via Limelight data
        } else if (gamepad2.y) {
            actuator.launchMotor(0);
        }

        //Update actual velocity every loop
        currentVelocity = actuator.getLaunchVelocity();
        //Check whether launcher is ready
        launcherReady = actuator.isLaunchReady(targetVelocity);

        //Display launcher telemetry
        telemetry.addData("Target Velocity", targetVelocity);
        telemetry.addData("Actual Velocity", currentVelocity);
        telemetry.addData("Launcher Ready", launcherReady);

        if (launcherReady) {
            actuator.ledIndicator.setPosition(0.44); //green
        }
        else {
            actuator.ledIndicator.setPosition(0.28); //red
        }
    }
    public void kicker() {
        //To get rid of the 4th pollen using the kicker servo, running on a FSM
        switch (kickerState) {
            case KICKER_START:
                if (gamepad2.rightBumperWasReleased()) {
                    actuator.kickerUp(); //change value in Actuators.java if needed
                    actuator.stepUpLeft.setPower(1);
                    actuator.stepUpRight.setPower(1);
                    kickerTimer.reset(); //start a timer
                    kickerState = KickerState.KICKER_UP; //move to the next case
                }
                break;
            case KICKER_UP:
                if (kickerTimer.seconds() > 0.4) {
                    actuator.kickerDown(); //change value in Actuators.java if needed
                    kickerTimer.reset(); //start a timer
                    kickerState = KickerState.KICKER_RESET; //reset back
                }
                break;
            case KICKER_RESET:
                if (kickerTimer.seconds() > 0.4) {
                    actuator.stepUpLeft.setPower(0);
                    actuator.stepUpRight.setPower(0);
                    kickerState = KickerState.KICKER_START; //reset back
                }
        }
    }

    public void limelightTelemetry() {
        LLResult result = sensor.getLimelightResult();
        if (result != null && result.isValid()) {
            telemetry.addData("Limelight Target", "TX: %.2f°, TY: %.2f°, TA: %.2f%%",
                    result.getTx(), result.getTy(), result.getTa());
        } else {
            telemetry.addData("Limelight Target", "No Target");
        }
    }
}
