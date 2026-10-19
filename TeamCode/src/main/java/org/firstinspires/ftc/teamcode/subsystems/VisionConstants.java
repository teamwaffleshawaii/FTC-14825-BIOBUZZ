package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;

/**
 * Limelight mounting measurements in robot coordinates.
 * These are calibration inputs only; they do not alter AprilTag pose data
 * or automatically control the launcher.
 */
@Config
public class VisionConstants {
    private VisionConstants() {}

    // Current fixed camera mount, measured at the lens center.
    public static double CAMERA_HEIGHT_IN = 3.0;
    public static double CAMERA_PITCH_DEG = 45.0;

    // Measure from robot center before using robot-relative target positions.
    public static double CAMERA_FORWARD_OFFSET_IN = 0.0;
    public static double CAMERA_LEFT_OFFSET_IN = 0.0;

    // Reserved for future turret-aware camera transforms.
    public static boolean CAMERA_ON_TURRET = false;
}
