package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/** Blue alliance driver controls; only 38-45 AprilTags are accepted. */
@TeleOp(name = "TeleOp BLUE", group = "TeleOp")
public class TeleOpBlue extends TeleOperated {
    @Override
    protected boolean isRedAlliance() {
        return false;
    }
}
