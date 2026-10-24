package com.example.meepmeeptesting;


import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;


public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(70, 70, Math.toRadians(180), Math.toRadians(180), 14.5)
                .build();
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(60, -11.5, Math.toRadians(180)))
                .strafeToLinearHeading(new Vector2d( 47, -11.5), Math.toRadians(180))
                .splineTo(new Vector2d(30, -44), Math.toRadians(270))
                .strafeTo(new Vector2d(25, -55))
                .strafeToLinearHeading(new Vector2d(25,-40), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-25,-58), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-54,-1.5), Math.toRadians(-5))
                .strafeToLinearHeading(new Vector2d(-30,-56), Math.toRadians(90))

                //.strafeToLinearHeading(new Vector2d(63, -56), Math.toRadians(270))
                // Run your test paths here
                //...
                //Examples below
                //.strafeTo(new Vector2d(59.9, -11.5))
                //.strafeToLinearHeading(new Vector2d(0,0), Math.toRadians(0))
                //.splineTo(new Vector2d(0, 0), Math.toRadians(0))
                //.splineToConstantHeading(new Vector2d(0,0), Math.toRadians(0))
                //.splineToLinearHeading(new Pose2d(0,0, Math.toRadians(0)), Math.toRadians(0))
                //.waitSeconds(0.5)
                .build());

        Image fieldImage;

        // Look for the image bundled with the simulator.
        // This works regardless of the student's username or project location.
        try (java.io.InputStream imageStream =
                     MeepMeepTesting.class.getResourceAsStream("/biobuzz.png")) {

            if (imageStream == null) {
                throw new IOException("biobuzz.png is missing from src/main/resources.");
            }

            fieldImage = ImageIO.read(imageStream);

            if (fieldImage == null) {
                throw new IOException("biobuzz.png could not be read as an image.");
            }
        } catch (IOException e) {
            throw new RuntimeException("Could not load the BIOBUZZ field image.", e);
        }

        // Display the field and start the simulation.
        meepMeep.setBackground(fieldImage)
                .setDarkMode(true)
                .setBackgroundAlpha(1.0f)
                .addEntity(myBot)
                .start();


    }
}
