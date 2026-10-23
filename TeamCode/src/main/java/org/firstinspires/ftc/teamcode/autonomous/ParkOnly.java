// Backup autonomous: drive 15 field-X units (inches) with Road Runner.
// Confirm this direction is forward for the robot's actual starting orientation.

package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.odometry.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Actuators;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Sensors;

@Autonomous (name = "Park Only", group = "Auto")
public class ParkOnly extends LinearOpMode {

    // References to shared Robot subsystems (not duplicate hardware).
    Robot robot;
    Actuators actuator;
    Sensors sensor;
    IntakeSubsystem intakeSubsystem;
    LauncherSubsystem launcher;

    @Override
    public void runOpMode() throws InterruptedException {
        // Robot initializes all shared hardware and subsystems.
        robot = new Robot(hardwareMap);
        actuator = robot.actuator;
        sensor = robot.sensor;
        intakeSubsystem = robot.intake;
        launcher = robot.launcher;

        // Starting pose must match the robot's real position and heading.
        // Starting pose is our best estimate of the robot's field position.
        // Road Runner uses it as the reference for subsequent trajectories.
        MecanumDrive drive = new MecanumDrive(hardwareMap,
                new Pose2d(0, 0, Math.toRadians(0) ) );

        // This backup has no launcher or collection sequence.
        // Starting at (0,0) is a relative reference, not a measured field location.
        // Positive field X must match the desired parking direction.
        Action park = drive.actionBuilder(new Pose2d(0, 0, Math.toRadians(0) ))
                .strafeToLinearHeading(new Vector2d(15,0), Math.toRadians(0))
                .build();

        waitForStart();

        try {
            // Only drive when PLAY is active; no shots or intake actions.
            if (opModeIsActive() && !isStopRequested()) {
                Actions.runBlocking(park);
            }
        } finally {
            // Stop hardware safely even if the driver presses STOP.
            robot.shutdown();
        }
    }
}
