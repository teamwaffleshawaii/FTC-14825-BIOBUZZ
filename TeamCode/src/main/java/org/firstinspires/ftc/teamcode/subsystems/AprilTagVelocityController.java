package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.Arrays;

/**
 * TeleOp flywheel target from fresh alliance-specific AprilTags.
 * Calibration is currently available ONLY for red tags 30-33.
 * Other accepted alliance tags are shown in telemetry but cannot change velocity
 * until their own TY calibration is measured. Never extrapolate outside measured TY.
 */
@Config
public class AprilTagVelocityController {
    public static boolean ENABLED = true;
    public static double SMOOTHING = 0.25; // 0..1, larger responds faster
    public static double LOST_TARGET_HOLD_SECONDS = 0.5;

    // Rows: red tag 30, 31, 32, 33; columns: position 1, 2, 3.
    // Calibration velocities: 1020, 1040, 1000 ticks/s respectively.
    private static final double[][] RED_TY = {
            {0.58, -5.01, 6.56},
            {0.58, -5.00, 6.47},
            {0.58, -4.98, 6.32},
            {0.56, -4.96, 6.21}
    };
    private static final double[] RED_SPEEDS = {1020, 1040, 1000};

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
        if (frame != null && redAlliance) {
            for (LLResultTypes.FiducialResult tag : frame.getFiducialResults()) {
                int id = tag.getFiducialId();
                double ty = tag.getTargetYDegrees();
                if (id >= 30 && id <= 33 && Double.isFinite(ty)) {
                    estimates[count++] = interpolate(RED_TY[id - 30], ty);
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
        if (observed <= ty[1]) return RED_SPEEDS[1];
        if (observed >= ty[2]) return RED_SPEEDS[2];
        if (observed <= ty[0]) {
            double t = (observed - ty[1]) / (ty[0] - ty[1]);
            return RED_SPEEDS[1] + t * (RED_SPEEDS[0] - RED_SPEEDS[1]);
        }
        double t = (observed - ty[0]) / (ty[2] - ty[0]);
        return RED_SPEEDS[0] + t * (RED_SPEEDS[2] - RED_SPEEDS[0]);
    }

    public int getCalibratedTagsSeen() { return calibratedTagsSeen; }
    public boolean hasRecentCalibration() {
        return hasValidMeasurement &&
                sinceValid.seconds() <= Math.max(0, LOST_TARGET_HOLD_SECONDS);
    }
}
