// Manual odometry mapping tool (TeleOp), kept here with autonomous helpers.
//You can use either MeepMeep, or this test code physically

package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.odometry.Drawing;
import org.firstinspires.ftc.teamcode.odometry.MecanumDrive;

/**
 * STUDENT GUIDE: Drive the robot manually to map autonomous coordinates.
 * Road Runner estimates field position using odometry/localization.
 * x and y describe position; heading describes robot rotation.
 * Dashboard draws a robot icon so we can visualize the estimated pose.
 */
@TeleOp
public class AutoMap extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Send the same telemetry to both Driver Station and FTC Dashboard.
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // This initial pose must match where the robot is actually placed.
        // Otherwise every reported coordinate will be offset.
        MecanumDrive drive = new MecanumDrive(hardwareMap,
                //TODO: Change robot starting position here as necessary, currently origin
                new Pose2d(-60, -11.5, Math.toRadians(0) ) );

        waitForStart();

        while (opModeIsActive()) {
            // Joysticks request forward/sideways motion and rotation.
            // 0.5 scales the commands to half of their usual magnitude.
            drive.setDrivePowers(new PoseVelocity2d(
                    //TODO: Reduce robot speed if needed
                    //Currently at 50% speed for more accurate driving
                    new Vector2d(
                            -gamepad1.left_stick_y * 0.5,
                            -gamepad1.left_stick_x * 0.5
                    ),
                    -gamepad1.right_stick_x * 0.5
            ));

            // Ask the localizer to integrate the latest odometry measurements.
            drive.updatePoseEstimate();

            // Read the current estimated position and heading.
            Pose2d pose = drive.localizer.getPose();
            telemetry.addData("x", pose.position.x);
            telemetry.addData("y", pose.position.y);
            telemetry.addData("heading (deg)", Math.toDegrees(pose.heading.toDouble()));
            telemetry.update();

            // Draw the estimated pose on Dashboard's virtual field overlay.
            // Hide Dashboard's built-in season field and draw our shared field image.
            TelemetryPacket packet = new TelemetryPacket(false);
            packet.fieldOverlay().drawImage("/images/biobuzz.png", 0, 0, 144, 144);
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), pose);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }
}
