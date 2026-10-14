package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Actuators {

    // TODO: If using motors, add/edit them here (up to 8)
    public DcMotor leftFront;
    public DcMotor rightFront;
    public DcMotor leftBack;
    public DcMotor rightBack;
    public DcMotor intakeMotor;
    public DcMotor transferMotor;
    public DcMotor launchMotor;

    // TODO: If using servos or continuous servos, add them here (up to 10)
    public Servo kickerServo;
    public CRServo stepUpRight;
    public CRServo stepUpLeft;
    public Servo intakeServoLeft;
    public Servo intakeServoRight;
    public Servo turretServo;


    public void init(HardwareMap hwMap) {
        // TODO: Put motor hardware here, don't forget to rename deviceNames to match yours
        leftFront = hwMap.get(DcMotor.class, "leftFront");
        rightFront = hwMap.get(DcMotor.class, "rightFront");
        leftBack = hwMap.get(DcMotor.class, "leftBack");
        rightBack = hwMap.get(DcMotor.class, "rightBack");
        intakeMotor = hwMap.get(DcMotor.class, "intakeMotor");
        transferMotor = hwMap.get(DcMotor.class, "transferMotor");
        launchMotor = hwMap.get(DcMotor.class, "launchMotor");

        //TODO: Set motor mode when power is zero
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //or FLOAT
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //or FLOAT
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //or FLOAT
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //or FLOAT
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //or FLOAT
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //or FLOAT
        launchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); //or BRAKE

        //TODO: If need to change motor direction
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE); //or FORWARD
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE); //or FORWARD
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD); //or REVERSE
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD); //or REVERSE
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD); //or REVERSE
        transferMotor.setDirection(DcMotorSimple.Direction.REVERSE); //or FORWARD
        launchMotor.setDirection(DcMotorSimple.Direction.REVERSE); //or FORWARD

        //TODO: Put servo hardware here, don't forget to rename deviceNames to match yours
        kickerServo = hwMap.get(Servo.class, "kickerServo");
        stepUpLeft = hwMap.get(CRServo.class, "stepUpLeft");
        stepUpRight = hwMap.get(CRServo.class, "stepUpRight");
        intakeServoRight = hwMap.get(Servo.class, "intakeServoRight");
        intakeServoLeft = hwMap.get(Servo.class, "intakeServoLeft");
        turretServo = hwMap.get(Servo.class, "turretServo");
        //TODO: If need to change servo direction
        stepUpLeft.setDirection(CRServo.Direction.REVERSE);
        stepUpRight.setDirection(CRServo.Direction.FORWARD);
        intakeServoRight.setDirection(Servo.Direction.FORWARD);
        intakeServoRight.setDirection(Servo.Direction.REVERSE);
        //TODO: If you need to set the logical min and max of a servo.
        //turretServo.scaleRange(0, 1);  //Change the 0 and 1 if needed
    }

    //TODO: Sample classes for turning on motors using power only (Useful for TeleOp)
    public void leftFront(double power) {
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setPower(power);
    }
    public void rightFront(double power) {
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setPower(power);
    }
    public void leftBack(double power) {
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setPower(power);
    }
    public void rightBack(double power) {
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setPower(power);
    }

    public void intakeMotor(double power) {
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setPower(power);
    }

    public void transferMotor(double power) {
        transferMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transferMotor.setPower(power);
    }

    public void launchMotor(double power) {
        launchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ((DcMotorEx) launchMotor).setVelocity(power);
    }

    //TODO: Sample classes for powering a servo
    public void stepUpOn() {
        stepUpLeft.setPower(1);
        stepUpRight.setPower(1);
    }
    public void stepUpOff() {
        stepUpLeft.setPower(0);
        stepUpRight.setPower(0);
    }

    public void kicker(double position) {
        kickerServo.setPosition(position);
    }

    public void intakeRampDown() {
        intakeServoLeft.setPosition(0.4);
        intakeServoRight.setPosition(0.4);
    }

    public void intakeRampUp() {
        intakeServoLeft.setPosition(0.6);
        intakeServoRight.setPosition(0.6);
    }

}
