package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/** BLUE alliance turret aiming; uses AprilTags 38-45. */
@TeleOp(name = "TeleOpTurret-BLUE", group = "TeleOp")
public class TeleOpTurretBlue extends TeleOpTurret {
    @Override
    protected boolean isRedAlliance() {
        return false;
    }
}
