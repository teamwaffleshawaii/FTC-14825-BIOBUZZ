package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Uses exactly the same driving, intake, ramp, and launcher controls as
 * competition TeleOp. Only the velocity source differs: this mode reads
 * TARGET_TICKS_PER_SECOND from FTC Dashboard, never from AprilTags.
 */
@Config
@TeleOp(name = "TeleOp-LauncherTuning", group = "Tuning")
public class TeleOpLauncherTuning extends TeleOperated {
    public static double TARGET_TICKS_PER_SECOND = 1020.0;

    private static final String TUNING_CODE_VERSION = "DIAG-2026-11-03-A";
    private double effectiveLauncherTarget = Double.NaN;

    @Override
    protected double getLaunchTargetVelocity() {
        double target = TARGET_TICKS_PER_SECOND;
        effectiveLauncherTarget = Double.isFinite(target) ? Math.max(0.0, target) : 0.0;
        return effectiveLauncherTarget;
    }
    @Override
    public void launcherTelemetry() {
        super.launcherTelemetry();
        telemetry.addData("Dashboard Config Target", TARGET_TICKS_PER_SECOND);
        telemetry.addData("Effective Launcher Target", effectiveLauncherTarget);
        telemetry.addData("Tuning Code Version", TUNING_CODE_VERSION);
    }
}
