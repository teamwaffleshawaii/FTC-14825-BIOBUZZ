package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Road Runner adapter for the shared launcher. Start the flywheel separately
 * and wait for readiness before executing this action.
 */
public class LauncherAction implements Action {
    private final LauncherSubsystem launcher;
    private boolean started = false;
    private final ElapsedTime timeout = new ElapsedTime();

    public LauncherAction(LauncherSubsystem launcher) {
        this.launcher = launcher;
    }

    @Override
    public boolean run(TelemetryPacket packet) {
        if (!started) {
            launcher.start();
            started = true;
            timeout.reset();
        }
        launcher.update();
        packet.put("Launch State", launcher.getState().name());
        packet.put("Distance (cm)", launcher.getLastDistanceCm());
        if (launcher.isBusy() && timeout.seconds() >= 5.0) {
            launcher.cancel();
            packet.put("Launch Timeout", true);
        }
        return launcher.isBusy();
    }
}
