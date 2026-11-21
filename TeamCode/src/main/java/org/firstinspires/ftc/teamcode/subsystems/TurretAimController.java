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

    // Field-tested mechanical turret center.
    public static double SERVO_CENTER = 0.48;
    // Servo position change per degree of horizontal aiming correction.
    public static double SERVO_PER_DEGREE = 0.010; //adjust this as needed if turret is not aligned center at all positions
    public static double TX_DEADBAND_DEGREES = 0.5;
    public static double SERVO_MIN = 0.00;
    public static double SERVO_MAX = 1.00;
    // Field-tested TX center bias for the current Limelight mounting position.
    // Tune in Dashboard only if automatic aim is consistently off-center.
    public static double TX_CENTER_OFFSET_DEGREES = -1.0; //adjust this as needed

    // Angle and Hypotenuse velocity compensation parameters
    public static boolean HYPOTENUSE_COMPENSATION_ENABLED = true;
    public static double HYPOTENUSE_SCALE = 1.15; // gentle factor for 1/cos(theta) distance increase
    public static double ANGLE_VELOCITY_PER_DEGREE = 1.0; // slight extra speed boost per degree off-center (ticks/sec)
    public static double MAX_ANGLE_VELOCITY_BOOST = 85.0; // cap on velocity boost from angled shots

    // Calibrated relative TX offsets for four adjacent tags; reused across RED and BLUE tag groups.
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

    /**
     * Returns the horizontal goal angle in degrees off-center from the fresh Limelight frame.
     * Returns 0.0 if no valid alliance AprilTags are found in the frame.
     */
    public static double getHorizontalAngleDegrees(LLResult frame, boolean redAlliance) {
        if (frame == null || !frame.isValid()) {
            return 0.0;
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
            return 0.0;
        }

        Arrays.sort(estimates, 0, count);
        double centerTx = (count % 2 == 1)
                ? estimates[count / 2]
                : (estimates[count / 2 - 1] + estimates[count / 2]) / 2.0;

        return centerTx - TX_CENTER_OFFSET_DEGREES;
    }

    /**
     * Calculates angle off-center in degrees from current turret servo position.
     */
    public static double getAngleFromServoPosition(double servoPosition) {
        if (!Double.isFinite(servoPosition) || SERVO_PER_DEGREE <= 0) return 0.0;
        return (SERVO_CENTER - servoPosition) / SERVO_PER_DEGREE;
    }

    /**
     * Calculates angle-compensated launch velocity taking into account the longer
     * hypotenuse distance and off-center turret launch angle.
     *
     * @param baseVelocity Velocity calculated from straight-line TY distance (ticks/sec)
     * @param txDegrees Horizontal offset angle off-center in degrees (from Limelight or turret)
     * @return Velocity compensated for hypotenuse distance increase and angled shot
     */
    public static double compensateLaunchVelocity(double baseVelocity, double txDegrees) {
        if (!Double.isFinite(baseVelocity) || baseVelocity <= 0) return baseVelocity;
        if (!Double.isFinite(txDegrees)) return baseVelocity;

        double absAngleDeg = Math.abs(txDegrees);
        if (absAngleDeg < 0.5) return baseVelocity; // negligible angle

        // 1. Hypotenuse distance factor: 1 / cos(angle)
        double angleRad = Math.toRadians(Math.min(60.0, absAngleDeg));
        double cosVal = Math.cos(angleRad);
        double hypotenuseFactor = cosVal > 0.1 ? (1.0 / cosVal) : 1.0;

        // Apply hypotenuse scaling to base velocity if enabled
        double compensatedVelocity = baseVelocity;
        if (HYPOTENUSE_COMPENSATION_ENABLED) {
            // Velocity required scales with hypotenuse distance factor
            compensatedVelocity = baseVelocity * (1.0 + (hypotenuseFactor - 1.0) * HYPOTENUSE_SCALE);
        }

        // 2. Additional linear angle boost per degree off-center (Dashboard tunable)
        double linearBoost = absAngleDeg * ANGLE_VELOCITY_PER_DEGREE;
        linearBoost = Math.min(Math.max(0, MAX_ANGLE_VELOCITY_BOOST), linearBoost);

        return compensatedVelocity + linearBoost;
    }
}
