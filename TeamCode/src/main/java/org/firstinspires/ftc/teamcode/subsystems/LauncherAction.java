package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * STUDENT GUIDE: Adapts LauncherSubsystem to a Road Runner Action.
 * Road Runner repeatedly calls run(packet) while it returns true.
 * The flywheel is spun up separately; this Action only handles feeding.
 * IMPORTANT: The current AutoRight intentionally starts volleys without
 * checking readiness. The launcher itself does not wait for target speed.
 * Road Runner adapter for the shared launcher. Start the flywheel separately
 * and wait for readiness before executing this action.
 */
public class LauncherAction implements Action {
    private final LauncherSubsystem launcher;
    // Track the first call so we only start the launcher once per action.
    private boolean started = false;
    private final ElapsedTime timeout = new ElapsedTime();

    public LauncherAction(LauncherSubsystem launcher) {
        this.launcher = launcher;
    }

    @Override
    /** One step of the action. Return true to continue, false to finish. */
    public boolean run(TelemetryPacket packet) {
        if (!started) {
            launcher.start();
            started = true;
            timeout.reset();
        }
        launcher.update();
        packet.put("Launch State", launcher.getState().name());
        packet.put("Distance (cm)", launcher.getLastDistanceCm());
        // Fail-safe: cancel a jammed or stalled sequence after five seconds.
        if (launcher.isBusy() && timeout.seconds() >= 5.0) {
            launcher.cancel();
            packet.put("Launch Timeout", true);
        }
        return launcher.isBusy();
    }
}
