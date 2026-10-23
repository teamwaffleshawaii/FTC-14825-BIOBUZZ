package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.Arrays;

/**
 * TeleOp flywheel target from fresh alliance-specific AprilTags.
 * The four scoring areas reuse the same left-to-right tag-position calibration:
 * RED 30-33 / 34-37 and BLUE 38-41 / 42-45.
 * This assumes the camera mounting and tag geometry are equivalent at each area.
 * Never extrapolate outside the measured TY range.
 */
@Config
public class AprilTagVelocityController {
    public static boolean ENABLED = true;
    public static double SMOOTHING = 0.25; // 0..1, larger responds faster
    public static double LOST_TARGET_HOLD_SECONDS = 0.5;

    // Rows: tag position left-to-right (0..3); columns: measured positions 1, 2, 3.
    // Reused for each group of four tags, as requested.
    // Calibration velocities: 1020, 1040, 1000 ticks/s respectively.
    private static final double[][] TAG_POSITION_TY = {
            {0.58, -5.01, 6.56},
            {0.58, -5.00, 6.47},
            {0.58, -4.98, 6.32},
            {0.56, -4.96, 6.21}
    };
    private static final double[] CALIBRATED_SPEEDS = {1020, 1040, 1000};

    private final boolean redAlliance;
    private final ElapsedTime sinceValid = new ElapsedTime();
    private double filteredVelocity = Double.NaN;
    private boolean hasValidMeasurement = false;
    private int calibratedTagsSeen = 0;

    public AprilTagVelocityController(boolean redAlliance) {
        this.redAlliance = redAlliance;
    }

    public boolean acceptsTag(int id) {
        return redAlliance ? (id >= 30 && id <= 37) : (id >= 38 && id <= 45);
    }

    /** Returns the smoothed target or baseline when no recent calibrated tag exists. */
    public double update(LLResult frame, double baseline, boolean freeze) {
        if (freeze) return Double.isFinite(filteredVelocity) ? filteredVelocity : baseline;
        double[] estimates = new double[4];
        int count = 0;
        if (frame != null) {
            for (LLResultTypes.FiducialResult tag : frame.getFiducialResults()) {
                int id = tag.getFiducialId();
                double ty = tag.getTargetYDegrees();
                if (acceptsTag(id) && Double.isFinite(ty) && count < estimates.length) {
                    // Every block of four IDs shares the same positional calibration.
                    int tagPosition = (id - 30) % 4;
                    estimates[count++] = interpolate(TAG_POSITION_TY[tagPosition], ty);
                }
            }
        }
        calibratedTagsSeen = count;
        if (count > 0) {
            Arrays.sort(estimates, 0, count);
            double measured = count % 2 == 1 ? estimates[count / 2]
                    : (estimates[count / 2 - 1] + estimates[count / 2]) / 2.0;
            double alpha = Math.max(0, Math.min(1, SMOOTHING));
            filteredVelocity = Double.isFinite(filteredVelocity)
                    ? filteredVelocity + alpha * (measured - filteredVelocity) : measured;
            sinceValid.reset();
            hasValidMeasurement = true;
        } else if (!hasValidMeasurement ||
                sinceValid.seconds() > Math.max(0, LOST_TARGET_HOLD_SECONDS)) {
            // Do not keep using a stale distance indefinitely.
            filteredVelocity = baseline;
            hasValidMeasurement = false;
        }
        return filteredVelocity;
    }

    /** Piecewise linear TY interpolation, clamped to the calibrated endpoints. */
    private static double interpolate(double[] ty, double observed) {
        // Measured TY decreases as required velocity increases.
        if (observed <= ty[1]) return CALIBRATED_SPEEDS[1];
        if (observed >= ty[2]) return CALIBRATED_SPEEDS[2];
        if (observed <= ty[0]) {
            double t = (observed - ty[1]) / (ty[0] - ty[1]);
            return CALIBRATED_SPEEDS[1] + t * (CALIBRATED_SPEEDS[0] - CALIBRATED_SPEEDS[1]);
        }
        double t = (observed - ty[0]) / (ty[2] - ty[0]);
        return CALIBRATED_SPEEDS[0] + t * (CALIBRATED_SPEEDS[2] - CALIBRATED_SPEEDS[0]);
    }

    public int getCalibratedTagsSeen() { return calibratedTagsSeen; }
    public boolean hasRecentCalibration() {
        return hasValidMeasurement &&
                sinceValid.seconds() <= Math.max(0, LOST_TARGET_HOLD_SECONDS);
    }
}
