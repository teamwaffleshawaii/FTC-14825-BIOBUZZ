package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class DriverControlled extends LinearOpMode {

    double reset_angle = 0;
    Actuators actuator = new Actuators();
    Sensors sensor = new Sensors();

    @Override
    public void runOpMode() {
        actuator.init(hardwareMap);
        sensor.init(hardwareMap);

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                drive();
                resetAngle();
                intake();
                launch();
                telemetry.update();
            }
        }
    }

    public void drive() {
        double Protate = - gamepad1.left_stick_x;
        double stick_x = - gamepad1.right_stick_x * Math.sqrt(Math.pow(1-Math.abs(Protate), 2) );
        //Accounts for Protate when limiting magnitude to be less than 1
        double stick_y = - gamepad1.right_stick_y * Math.sqrt(Math.pow(1-Math.abs(Protate), 2) );
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

        telemetry.addData("Stick_X", stick_x);
        telemetry.addData("Stick_Y", stick_y);
        telemetry.addData("Magnitude",  Math.sqrt(Math.pow(stick_x, 2) + Math.pow(stick_y, 2)));
        telemetry.addData("Motor Front Left", Py - Protate);
        telemetry.addData("Motor Front Right", Px + Protate);
        telemetry.addData("Motor Back Left", Px - Protate);
        telemetry.addData("Motor Back Right", Py + Protate);

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

    public void intake() {
        if (gamepad1.a) {
            actuator.intakeMotor.setPower(1);
            actuator.transferMotor.setPower(1);
        } else if (gamepad1.b) {
            actuator.intakeMotor.setPower(-0.8);
            actuator.transferMotor.setPower(-0.8);
        } else if (gamepad1.right_bumper) {
            actuator.stepUpLeft.setPower(1);
            actuator.stepUpRight.setPower(1);
            actuator.intakeMotor.setPower(1);
            actuator.transferMotor.setPower(1);
        } else {
            actuator.stepUpLeft.setPower(0);
            actuator.stepUpRight.setPower(0);
            actuator.intakeMotor.setPower(0);
            actuator.transferMotor.setPower(0);
        }
    }

    public void launch() {
        if (gamepad1.right_trigger > 0.2) {
            actuator.launchMotor(850);
        } else {
            actuator.launchMotor(0);
        }

        //To get rid of the 4th pollen using the kicker servo
        if (gamepad1.rightBumperWasReleased()) {
            actuator.kickerServo.setPosition(0);
            sleep(500);
            actuator.kickerServo.setPosition(0.5);
        }
    }
}
