package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import java.util.Arrays;

/**
 * Shared AprilTag turret aiming controller for TeleOp and Autonomous.
 *
 * Uses Limelight 3A detected AprilTags (RED 30-37, BLUE 38-45) to calculate
 * goal center horizontal offset (TX) and compute required turret servo position.
 */
@Config
public class TurretAimController {

    public static double SERVO_CENTER = 0.46;
    // Increasing servo position turns LEFT. Positive TX points RIGHT.
    public static double SERVO_PER_DEGREE = 0.016;
    public static double TX_DEADBAND_DEGREES = 0.5;
    public static double SERVO_MIN = 0.00;
    public static double SERVO_MAX = 1.00;
    public static double TX_CENTER_OFFSET_DEGREES = 0.0;

    // Measured RED 30-33 TX offsets, provisionally reused for both RED and BLUE tag groups.
    public static double[] TAG_TX_OFFSETS = {-7.04, -3.02, 2.96, 7.02};

    private TurretAimController() {}

    /**
     * Calculates required turret servo position based on fresh Limelight AprilTag frame.
     * Returns null if no valid alliance AprilTags are found in the frame.
     */
    public static Double calculateTurretServoPosition(LLResult frame, boolean redAlliance) {
        if (frame == null || !frame.isValid()) {
            return null;
        }

        int firstTagId = redAlliance ? 30 : 38;
        int lastTagId = redAlliance ? 37 : 45;
        double[] estimates = new double[8];
        int count = 0;

        for (LLResultTypes.FiducialResult tag : frame.getFiducialResults()) {
            int id = tag.getFiducialId();
            double tx = tag.getTargetXDegrees();
            if (id >= firstTagId && id <= lastTagId && Double.isFinite(tx) && count < estimates.length) {
                estimates[count++] = tx - TAG_TX_OFFSETS[(id - firstTagId) % 4];
            }
        }

        if (count == 0) {
            return null;
        }

        Arrays.sort(estimates, 0, count);
        double centerTx = (count % 2 == 1)
                ? estimates[count / 2]
                : (estimates[count / 2 - 1] + estimates[count / 2]) / 2.0;

        double error = centerTx - TX_CENTER_OFFSET_DEGREES;
        double correction = Math.abs(error) <= TX_DEADBAND_DEGREES ? 0.0 : error;
        double min = Math.max(0.0, Math.min(1.0, SERVO_MIN));
        double max = Math.max(min, Math.min(1.0, SERVO_MAX));

        return Math.max(min, Math.min(max, SERVO_CENTER - correction * SERVO_PER_DEGREE));
    }
}
