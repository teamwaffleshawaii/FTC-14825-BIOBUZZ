package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.Arrays;

/**
 * Experimental turret aiming for RED; BLUE extends this shared implementation.
 * Driving, intake, launcher velocity and ball feeding remain unchanged.
 * Starts with automatic aiming enabled; test with balls removed.
 */
@Config
@TeleOp(name = "TeleOpTurret-RED", group = "TeleOp")
public class TeleOpTurretRed extends TeleOperated {
    // Automatic aiming is enabled for this experimental OpMode.
    // Set false in Dashboard to return to telemetry-only testing.
    public static boolean AUTO_AIM_ENABLED = true;

    public static double SERVO_CENTER = 0.46;
    // Increasing servo position turns LEFT. Positive TX points RIGHT.
    // 0.04 servo position / approximately 2.5 degrees.
    public static double SERVO_PER_DEGREE = 0.016;
    public static double TX_DEADBAND_DEGREES = 0.5;
    // Full servo command range; verify safe physical travel on the robot.
    public static double SERVO_MIN = 0.00;
    public static double SERVO_MAX = 1.00;
    // Compensate if camera optical center is not turret-forward.
    public static double TX_CENTER_OFFSET_DEGREES = 0.0;

    // Measured RED 30-33 TX offsets, provisionally reused for both RED and BLUE
    // tag groups. Validate on the field; perspective and distance affect TX.
    private static final double[] TAG_TX_OFFSETS = {-7.04, -3.02, 2.96, 7.02};

    @Override
    protected boolean isRedAlliance() { return true; }

    @Override
    public void limelightTelemetry() {
        super.limelightTelemetry(); // Keep original TeleOp RED telemetry.

        LLResult frame = getTurretAprilTagFrame(); // Already freshness-checked.
        int firstTagId = isRedAlliance() ? 30 : 38;
        int lastTagId = isRedAlliance() ? 37 : 45;
        double[] estimates = new double[8];
        int count = 0;
        if (frame != null) {
            for (LLResultTypes.FiducialResult tag : frame.getFiducialResults()) {
                int id = tag.getFiducialId();
                double tx = tag.getTargetXDegrees();
                if (id >= firstTagId && id <= lastTagId && Double.isFinite(tx)
                        && count < estimates.length) {
                    // Estimate goal-center TX rather than aiming at one tag.
                    estimates[count++] = tx - TAG_TX_OFFSETS[(id - firstTagId) % 4];
                }
            }
        }

        telemetry.addData("Turret Mode",
                AUTO_AIM_ENABLED ? "AUTO AIM" : "TELEMETRY ONLY");
        telemetry.addData("Turret Tags Used", count);
        if (count == 0) {
            telemetry.addData("Turret Aim", "No fresh alliance tags; holding servo");
            return;
        }

        // Median rejects some outliers when multiple tags are visible.
        Arrays.sort(estimates, 0, count);
        double centerTx = count % 2 == 1 ? estimates[count / 2]
                : (estimates[count / 2 - 1] + estimates[count / 2]) / 2.0;
        double error = centerTx - TX_CENTER_OFFSET_DEGREES;
        double correction = Math.abs(error) <= TX_DEADBAND_DEGREES ? 0.0 : error;
        double min = Math.max(0.0, Math.min(1.0, SERVO_MIN));
        double max = Math.max(min, Math.min(1.0, SERVO_MAX));
        // Camera is stationary: calculate ABSOLUTE servo position from center,
        // not repeated incremental corrections that would cause drifting.
        double target = Math.max(min, Math.min(max,
                SERVO_CENTER - correction * SERVO_PER_DEGREE));

        telemetry.addData("Turret Center TX (deg)", centerTx);
        telemetry.addData("Turret TX Error (deg)", error);
        telemetry.addData("Turret Suggested Servo", target);
        telemetry.addData("Turret Actual Servo", actuator.turretServo.getPosition());

        if (AUTO_AIM_ENABLED && !launcher.isBusy()) {
            actuator.turretServo.setPosition(target);
        }
    }
}
