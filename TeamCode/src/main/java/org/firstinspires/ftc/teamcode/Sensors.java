package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Sensors {
    public IMU imu;

    // TODO: If using a 2M Distance Sensor, add them here
    public DistanceSensor distanceSensor;
    public Limelight3A limelight;

    public void init(HardwareMap hwMap) {
        //TODO: If using distance sensors
        distanceSensor = hwMap.get(DistanceSensor.class, "distanceSensor");

        // Initialize IMU
        try {
            imu = hwMap.get(IMU.class, "imu");
            RevHubOrientationOnRobot RevOrientation =
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                            RevHubOrientationOnRobot.UsbFacingDirection.UP);
            imu.initialize(new IMU.Parameters(RevOrientation));
        } catch (Exception e) {
            // IMU not found/configured
        }

        // Initialize Limelight 3A (Hardware config name: "limelight")
        try {
            limelight = hwMap.get(Limelight3A.class, "limelight");
            limelight.pipelineSwitch(0);
            limelight.start();
        } catch (Exception e) {
            // Limelight not found/configured
        }
    }

    public double getHeading(AngleUnit angleUnit) {
        if (imu != null) {
            return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        }
        return 0.0;
    }

    //TODO: Sample class for measuring distance from the distance sensor
    public double getDistance(DistanceUnit du) {
        return distanceSensor.getDistance(du);
    }

    public LLResult getLimelightResult() {
        if (limelight != null) {
            return limelight.getLatestResult();
        }
        return null;
    }

    public LLStatus getLimelightStatus() {
        if (limelight != null) {
            return limelight.getStatus();
        }
        return null;
    }

    public void setLimelightPipeline(int pipelineIndex) {
        if (limelight != null) {
            limelight.pipelineSwitch(pipelineIndex);
        }
    }

    public void stopLimelight() {
        if (limelight != null) {
            limelight.stop();
        }
    }
}
