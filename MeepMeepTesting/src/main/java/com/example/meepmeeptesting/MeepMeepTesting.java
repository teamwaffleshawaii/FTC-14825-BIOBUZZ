package com.example.meepmeeptesting;


import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 14.5)
                .build();
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(60, -14, Math.toRadians(180)))
                // Run your test paths here
                //...
                //Examples below
                //.strafeTo(new Vector2d(0, 0))
                //.strafeToLinearHeading(new Vector2d(0,0), Math.toRadians(0))
                //.splineTo(new Vector2d(0, 0), Math.toRadians(0))
                //.splineToConstantHeading(new Vector2d(0,0), Math.toRadians(0))
                //.splineToLinearHeading(new Pose2d(0,0, Math.toRadians(0)), Math.toRadians(0))
                //.waitSeconds(0.5)
                .build());
        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
