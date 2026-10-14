package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//If using navX sensor (needs external libraries)
//import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
//import com.kauailabs.navx.ftc.AHRS;

public class Sensors {
    public IMU imu;

    public void init(HardwareMap hwMap) {
        //TODO: Put sensor hardware here, don't forget to rename deviceNames to match yours

        //TODO: If using the IMU
        imu = hwMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot RevOrientation =
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.
                        LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP);
            imu.initialize(new IMU.Parameters(RevOrientation));
    }

    //TODO: Sample class for measuring the REV Control Hub heading through the IMU
    public double getHeading(AngleUnit angleUnit) {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES); // if using the imu
    }

}
