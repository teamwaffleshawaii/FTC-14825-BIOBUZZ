package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Run an existing autonomous Road Runner action while updating both LED indicators.
 * This is display-only: it never waits for flywheel readiness or changes the action.
 */
public final class AutoLedAction {
    private AutoLedAction() {}

    public static void runBlocking(Actuators actuator, LauncherSubsystem launcher, Action action) {
        ElapsedTime ledTimer = new ElapsedTime();
        Actions.runBlocking(packet -> {
            boolean running = action.run(packet);
            double target = launcher.getCommandedFlywheelVelocity();
            boolean ready = actuator.isLaunchReady(target);
            if (ready) {
                // Match TeleOp: 150 ms green, 150 ms off.
                if ((ledTimer.milliseconds() % 300) < 150) {
                    actuator.indicatorGreen();
                } else {
                    actuator.indicatorOff();
                }
            } else {
                actuator.indicatorRed();
            }
            packet.put("Launcher Ready", ready);
            return running;
        });
    }
}
