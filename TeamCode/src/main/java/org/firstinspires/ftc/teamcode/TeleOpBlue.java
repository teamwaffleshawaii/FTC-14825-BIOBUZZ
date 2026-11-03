package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/** BLUE alliance turret aiming; uses AprilTags 38-45. */
@TeleOp(name = "TeleOp BLUE", group = "TeleOp")
public class TeleOpBlue extends TeleOpRed {
    @Override
    protected boolean isRedAlliance() {
        return false;
    }
}
