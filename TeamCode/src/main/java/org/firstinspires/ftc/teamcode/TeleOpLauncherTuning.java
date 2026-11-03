package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

/**
 * Standalone launcher tuning: NO AprilTag target adjustment, turret auto-aim,
 * or drivetrain commands. Use with balls removed for initial PIDF testing.
 * Dashboard: tune LauncherSubsystem.LAUNCH_P/I/D/F and TARGET_TICKS_PER_SECOND.
 * Dashboard changes are temporary until copied into Java defaults.
 */
@Config
@TeleOp(name = "TeleOp-LauncherTuning", group = "Tuning")
public class TeleOpLauncherTuning extends LinearOpMode {
    public static double TARGET_TICKS_PER_SECOND = 1020.0;
    public static double MAX_TARGET_TICKS_PER_SECOND = 1500.0;

    private static double clampTarget(double value) {
        if (!Double.isFinite(value)) return 0;
        return Math.max(0, Math.min(MAX_TARGET_TICKS_PER_SECOND, value));
    }

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Robot robot = new Robot(hardwareMap);
        boolean flywheelOn = false;
        boolean previousA = false, previousUp = false, previousDown = false;
        boolean previousLeft = false, previousRight = false, previousReset = false;
        boolean previousFeed = false;
        try {
            // Prevent unintended motion before PLAY.
            robot.actuator.launchMotor(0);
            robot.drivetrain.stop();
            robot.intake.stopFeeding();
            robot.sensor.stopLimelight(); // Tuning never needs Limelight.
            telemetry.addLine("Launcher tuning only: drivetrain and auto-aim disabled.");
            telemetry.addLine("G1 A: flywheel toggle; B: stop; Y: reset target.");
            telemetry.addLine("G1 D-pad up/down: +/-10; right/left: +/-1.");
            telemetry.addLine("G2 right bumper: start feed; G2 Y: cancel feed + stop flywheel.");
            telemetry.addLine("Tune PIDF in Dashboard under LauncherSubsystem.");
            telemetry.update();
            waitForStart();
            while (opModeIsActive()) {
                // Edge-trigger changes: a held button changes target only once.
                if (gamepad1.dpad_up && !previousUp) TARGET_TICKS_PER_SECOND += 10;
                if (gamepad1.dpad_down && !previousDown) TARGET_TICKS_PER_SECOND -= 10;
                if (gamepad1.dpad_right && !previousRight) TARGET_TICKS_PER_SECOND += 1;
                if (gamepad1.dpad_left && !previousLeft) TARGET_TICKS_PER_SECOND -= 1;
                if (gamepad1.y && !previousReset) TARGET_TICKS_PER_SECOND = 1020;
                previousUp = gamepad1.dpad_up;
                previousDown = gamepad1.dpad_down;
                previousRight = gamepad1.dpad_right;
                previousLeft = gamepad1.dpad_left;
                previousReset = gamepad1.y;
                TARGET_TICKS_PER_SECOND = clampTarget(TARGET_TICKS_PER_SECOND);

                if (gamepad1.a && !previousA) flywheelOn = !flywheelOn;
                previousA = gamepad1.a;
                if (gamepad1.b || gamepad2.y) {
                    flywheelOn = false;
                    robot.launcher.cancel();
                }

                // Do not allow a new feed until the bumper is released and pressed again.
                if (!gamepad2.y && gamepad2.right_bumper && !previousFeed
                        && flywheelOn && !robot.launcher.isBusy()) {
                    robot.launcher.start();
                }
                previousFeed = gamepad2.right_bumper;

                robot.launcher.update(); // Applies live Dashboard PIDF edits.
                double target = flywheelOn ? TARGET_TICKS_PER_SECOND : 0;
                robot.launcher.setFlywheelVelocity(target);
                double actual = robot.launcher.getActualFlywheelVelocity();

                telemetry.addData("Flywheel", flywheelOn ? "ON" : "OFF");
                telemetry.addData("Target Velocity", target);
                telemetry.addData("Actual Velocity", actual);
                telemetry.addData("Velocity Error", flywheelOn ? target - actual : 0);
                telemetry.addData("Launcher Ready", flywheelOn && robot.actuator.isLaunchReady(target));
                telemetry.addData("P", LauncherSubsystem.LAUNCH_P);
                telemetry.addData("I", LauncherSubsystem.LAUNCH_I);
                telemetry.addData("D", LauncherSubsystem.LAUNCH_D);
                telemetry.addData("F", LauncherSubsystem.LAUNCH_F);
                telemetry.addData("Launch State", robot.launcher.getState());
                telemetry.addLine("Limelight velocity adjustment: DISABLED");
                telemetry.update();
            }
        } finally {
            robot.shutdown(); // Includes explicit flywheel motor stop.
        }
    }
}
