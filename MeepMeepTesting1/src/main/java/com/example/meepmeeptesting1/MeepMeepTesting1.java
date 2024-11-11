package com.example.meepmeeptesting1;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
public class MeepMeepTesting1 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(0, -63, Math.toRadians(90)))
                .waitSeconds(3)
                .strafeTo(new Vector2d(0,-31))
                .waitSeconds(1.5)

                .setTangent(180)
                .splineToLinearHeading(new Pose2d(-48.5, -40, Math.toRadians(90)), Math.toRadians(90))
                .waitSeconds(2.5)
                //.strafeToLinearHeading(new Vector2d(-48, -46), Math.toRadians(45))
                .setTangent(180)
                .splineToLinearHeading(new Pose2d(-57.5, -57.5, Math.toRadians(230)), Math.toRadians(230))
                //.strafeToLinearHeading(new Vector2d(-57.5, -57.5), Math.toRadians(230))
                .waitSeconds(2)
                .strafeToLinearHeading(new Vector2d(-58.5, -40), Math.toRadians(90))
                .waitSeconds(2)
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(-57.5, -57.5, Math.toRadians(230)), Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(-56, -26), Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(-57.5, -57.5), Math.toRadians(230))

                .splineToLinearHeading(new Pose2d(-26, -10, Math.toRadians(0)), Math.toRadians(-25))
                        .build());
        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}