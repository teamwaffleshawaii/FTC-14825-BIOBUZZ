package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/** Red alliance driver controls; only 30-37 AprilTags are accepted. */
@TeleOp(name = "TeleOp RED", group = "TeleOp")
public class TeleOpRed extends TeleOperated {
    @Override
    protected boolean isRedAlliance() {
        return true;
    }
}
