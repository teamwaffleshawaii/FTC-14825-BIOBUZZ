package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * STUDENT GUIDE: Hardware access layer for motors and servos.
 * HardwareMap names MUST match the robot configuration exactly.
 * Higher-level subsystems decide WHEN and WHY to move each mechanism;
 * this class exposes the low-level hardware commands.
 */
public class Actuators {

    // TODO: If using motors, add/edit them here (up to 8)
    public DcMotor leftFront;
    public DcMotor rightFront;
    public DcMotor leftBack;
    public DcMotor rightBack;
    public DcMotor intakeMotor;
    public DcMotorEx transferMotor;
    // DcMotorEx adds encoder velocity methods used by flywheel PIDF.
    public DcMotorEx launchMotor;

    // TODO: If using servos or continuous servos, add them here (up to 10)
    public Servo kickerServo;
    public CRServo stepUpRight;
    public CRServo stepUpLeft;
    public Servo intakeServoLeft;
    public Servo intakeServoRight;
    public Servo turretServo;
    public Servo ledIndicator;

    /** Called once from Robot before any mechanism is used. */
    public void init(HardwareMap hwMap) {
        // TODO: Put motor hardware here, don't forget to rename deviceNames to match yours
        leftFront = hwMap.get(DcMotor.class, "leftFront");
        rightFront = hwMap.get(DcMotor.class, "rightFront");
        leftBack = hwMap.get(DcMotor.class, "leftBack");
        rightBack = hwMap.get(DcMotor.class, "rightBack");
        intakeMotor = hwMap.get(DcMotor.class, "intakeMotor");
        transferMotor = hwMap.get(DcMotorEx.class, "transferMotor");
        launchMotor = hwMap.get(DcMotorEx.class, "launchMotor");

        // Keep the encoder mode here; LauncherSubsystem owns PIDF tuning.
        // Encoder feedback enables velocity control; LauncherSubsystem sets PIDF.
        launchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //TODO: Set motor mode when power is zero
        // BRAKE resists motion at zero power; FLOAT allows coasting.
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //or FLOAT
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //or FLOAT
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //or FLOAT
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //or FLOAT
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //or FLOAT
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //or FLOAT
        launchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); //or BRAKE

        //TODO: If need to change motor direction
        // Opposite sides of a drivetrain may need reversed directions
        // so a positive command moves the robot the intended way.
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE); //or FORWARD
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE); //or FORWARD
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD); //or REVERSE
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD); //or REVERSE
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD); //or REVERSE
        transferMotor.setDirection(DcMotorSimple.Direction.REVERSE); //or FORWARD
        launchMotor.setDirection(DcMotorSimple.Direction.REVERSE); //or FORWARD

        //TODO: Put servo hardware here, don't forget to rename deviceNames to match yours
        // Standard Servo uses a position (0.0-1.0); CRServo uses motor-like power.
        kickerServo = hwMap.get(Servo.class, "kickerServo");
        stepUpLeft = hwMap.get(CRServo.class, "stepUpLeft");
        stepUpRight = hwMap.get(CRServo.class, "stepUpRight");
        intakeServoRight = hwMap.get(Servo.class, "intakeServoRight");
        intakeServoLeft = hwMap.get(Servo.class, "intakeServoLeft");
        turretServo = hwMap.get(Servo.class, "turretServo");
        // Center the turret at initialization for TeleOp and all Robot-based autos.
        turretServo.setPosition(0.46);
        ledIndicator = hwMap.get(Servo.class, "indicatorLED");
        //TODO: If need to change servo direction
        stepUpLeft.setDirection(CRServo.Direction.REVERSE);
        stepUpRight.setDirection(CRServo.Direction.FORWARD);
        intakeServoRight.setDirection(Servo.Direction.FORWARD);
        intakeServoLeft.setDirection(Servo.Direction.REVERSE);
        //TODO: If you need to set the logical min and max of a servo.
        //turretServo.scaleRange(0, 1);  //Change the 0 and 1 if needed
    }

    //TODO: Sample classes for turning on motors using power only (Useful for TeleOp)
    // These four methods drive wheels with power rather than encoder velocity.
    // RUN_WITHOUT_ENCODER does not use built-in velocity PIDF.
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

    /** Command flywheel speed in encoder ticks/second, not RPM. */
    public void launchMotor(double velocity) {
        launchMotor.setVelocity(velocity);
    }

    /** Read actual measured flywheel speed from the motor encoder. */
    public double getLaunchVelocity() {
        return launchMotor.getVelocity();
    }

    /** Only declare ready when the target is positive and error <= 50 ticks/s. */
    public boolean isLaunchReady(double targetVelocity) {
        double tolerance = 50.0; //ticks per second
        return targetVelocity > 0 &&
                Math.abs(getLaunchVelocity() - targetVelocity) <= tolerance;
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

    // Kicker positions are calibrated servo positions for this robot.
    public void kickerUp() {
        kickerServo.setPosition(0.03);
    }

    public void kickerDown() {
        kickerServo.setPosition(0.60);
    }

    // The left/right ramp servos move together to open or close the ramp.
    public void intakeRampDown() {
        intakeServoLeft.setPosition(0.5);
        intakeServoRight.setPosition(0.5);
    }

    public void intakeRampUp() {
        intakeServoLeft.setPosition(0.0);
        intakeServoRight.setPosition(0.0);
    }

}
