package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

/**
 * Road Runner adapter for the shared launcher. Start the flywheel separately
 * and wait for readiness before executing this action.
 */
public class LauncherAction implements Action {
    private final LauncherSubsystem launcher;
    private boolean started = false;

    public LauncherAction(LauncherSubsystem launcher) {
        this.launcher = launcher;
    }

    @Override
    public boolean run(TelemetryPacket packet) {
        if (!started) {
            launcher.start();
            started = true;
        }
        launcher.update();
        packet.put("Launch State", launcher.getState().name());
        packet.put("Distance (cm)", launcher.getLastDistanceCm());
        return launcher.isBusy();
    }
}
