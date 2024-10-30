package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(17.55, 17)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(0, -63,Math.toRadians(90)))
                        .forward(23)
                        //score specimen w claw
                        .lineToLinearHeading(new Pose2d(36, -40, Math.toRadians(180)))
                        .lineToLinearHeading(new Pose2d(36, -10, Math.toRadians(180)))
                        .lineToLinearHeading(new Pose2d(46, -10, Math.toRadians(180)))
                        .lineToLinearHeading(new Pose2d(46, -50, Math.toRadians(180)))
                        .lineToLinearHeading(new Pose2d(61, -50, Math.toRadians(180)))


                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}