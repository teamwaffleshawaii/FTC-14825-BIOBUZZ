package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import java.util.List;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * STUDENT GUIDE: Hardware sensors and camera access live in this class.
 *
 * This class reads measurements; it does NOT decide how fast to launch.
 * TeleOp's AprilTagVelocityController uses validated frames to estimate
 * flywheel velocity; this class only checks sensor/frame validity.
 */
public class Sensors {
    public IMU imu;

    // Sensor names must match the robot's hardware configuration.
    public DistanceSensor distanceSensor;
    public ColorSensor colorSensor;
    public Limelight3A limelight;

    /**
     * Run once during robot setup. The names below must match the names in
     * the REV Driver Station hardware configuration exactly.
     */
    public void init(HardwareMap hwMap) {
        distanceSensor = hwMap.get(DistanceSensor.class, "distanceSensor");
        colorSensor = hwMap.get(ColorSensor.class, "colorSensor");

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

        // The Limelight 3A processes camera images on the camera itself.
        // Pipeline 0 must be configured as an AprilTag pipeline in Limelight.
        // start() begins the SDK's periodic result polling.
        // If hardware is missing, the rest of the robot can still initialize.
        // Initialize Limelight 3A (Hardware config name: "limelight")
        try {
            limelight = hwMap.get(Limelight3A.class, "limelight");
            limelight.pipelineSwitch(APRILTAG_PIPELINE);
            limelight.start();
        } catch (Exception e) {
            // Limelight not found/configured
        }
    }

    /** Robot yaw (rotation around vertical axis), in requested angle units. */
    public double getHeading(AngleUnit angleUnit) {
        if (imu != null) {
            return imu.getRobotYawPitchRollAngles().getYaw(angleUnit);
        }
        return 0.0;
    }

    // Raw color channels are available for future diagnostics.
    public int getAmountRed() {return colorSensor.red();}
    public int getAmountGreen() {return colorSensor.green();}
    public int getAmountBlue() {return colorSensor.blue();}

    /** Normalized alpha is near 0.89 for a yellow ball and 0.039 for empty. */
    public double getIntakeColorAlpha() {
        if (colorSensor instanceof NormalizedColorSensor) {
            return ((NormalizedColorSensor) colorSensor).getNormalizedColors().alpha;
        }
        // If a non-normalized color sensor is installed, disable counting safely.
        return Double.NaN;
    }

    // Distance reading used to determine when the launcher feed is empty.
    public double getDistance(DistanceUnit du) {
        return distanceSensor.getDistance(du);
    }

    // CAMERA SAFETY: A valid target in an OLD frame is not a current target.
    // 250 ms is an initial freshness limit to test, not a universal rule.
    // Pipeline 0 is assumed to be configured for AprilTag detection.
    // Reject stale frames and non-AprilTag pipelines before using vision for targeting.
    public static final long MAX_LIMELIGHT_AGE_MS = 250;
    public static final int APRILTAG_PIPELINE = 0;

    /**
     * Returns a camera frame only if it is suitable for AprilTag telemetry.
     * Returns null when the camera has no trustworthy current tag result.
     *
     * A result may contain MULTIPLE tags. We have not yet chosen which tag
     * belongs to the scoring goal, so do not use an arbitrary tag for shooting.
     */
    public LLResult getFreshAprilTagResult() {
        // First ask for the most recently received frame.
        LLResult result = getLimelightResult();
        if (result == null || !result.isValid() ||
                result.getPipelineIndex() != APRILTAG_PIPELINE ||
                result.getStaleness() < 0 ||
                result.getStaleness() > MAX_LIMELIGHT_AGE_MS) {
            return null;
        }
        // FiducialResult represents one detected AprilTag (including its ID).
        // A frame without any tags should not be treated as a target lock.
        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        return tags != null && !tags.isEmpty() ? result : null;
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

    /** Switch processing pipelines; AprilTag validation expects pipeline 0. */
    public void setLimelightPipeline(int pipelineIndex) {
        if (limelight != null) {
            limelight.pipelineSwitch(pipelineIndex);
        }
    }

    /** Stop camera polling when the OpMode ends to free resources. */
    public void stopLimelight() {
        if (limelight != null) {
            limelight.stop();
        }
    }
}
