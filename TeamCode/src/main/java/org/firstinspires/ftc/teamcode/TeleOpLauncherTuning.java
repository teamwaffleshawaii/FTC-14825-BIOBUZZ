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
    public static double TARGET_TICKS_PER_SECOND = 1035.0;

    @Override
    protected double getLaunchTargetVelocity() {
        double target = TARGET_TICKS_PER_SECOND;
        if (!Double.isFinite(target)) return 0.0;
        return Math.max(0.0, target);
    }
}
