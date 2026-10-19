package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;

/**
 * STUDENT GUIDE: Physical measurements for the Limelight 3A camera.
 *
 * A camera sees the field from its own point of view. To calculate where a
 * detected AprilTag is relative to the ROBOT, we must also know where the
 * camera is mounted on the robot. These values describe that mounting.
 *
 * IMPORTANT: These numbers are currently stored for future calculations.
 * Nothing in this class transforms AprilTag coordinates or controls motors yet.
 * FTC Dashboard's @Config lets us edit the numbers while testing; changes
 * made in Dashboard do not permanently edit the Java source file.
 */
@Config
public class VisionConstants {
    // Utility class: all values are static, so we never create an object here.
    private VisionConstants() {}

    // Height is measured from the FLOOR to the CENTER of the camera lens.
    // Pitch is measured from horizontal: 0 = level, +45 = pointing upward.
    // These are current approximate measurements, not precise calibration.
    public static double CAMERA_HEIGHT_IN = 3.0;
    public static double CAMERA_PITCH_DEG = 45.0;

    // Offsets are measured from the ROBOT'S chosen reference point.
    // Positive forward = toward the front; positive left = robot's left.
    // Zero means 'not measured yet', NOT that the lens is at robot center.
    // Measure these before using a camera-to-robot pose transformation.
    public static double CAMERA_FORWARD_OFFSET_IN = 0.0;
    public static double CAMERA_LEFT_OFFSET_IN = 0.0;

    // Future turret mounting will change camera direction as the turret turns.
    // Setting this true by itself does NOT apply turret rotation corrections.
    // We will need a measured turret angle and camera-to-turret offsets first.
    public static boolean CAMERA_ON_TURRET = false;
}
