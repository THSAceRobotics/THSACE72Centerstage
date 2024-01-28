package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.profile.VelocityConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.net.PortUnreachableException;
import java.util.Vector;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(45, 45, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-33, 72-9, Math.toRadians(270)))
                                .lineToLinearHeading(new Pose2d(-35-9/Math.sqrt(2), 24+(9/Math.sqrt(2)),Math.toRadians(315)))
                                .addTemporalMarker(()->{})
                                .back(15)
                                .setTangent(Math.PI+1.3613902)
                                .splineToLinearHeading(new Pose2d(-16, 6, Math.toRadians(70)), Math.toRadians(0))
                                .lineToLinearHeading(new Pose2d(20, 6, Math.toRadians(110)))
                                .setTangent(Math.toRadians(0))
                                .splineToLinearHeading(new Pose2d(51, 28, Math.toRadians(180)), Math.toRadians(90))
                                .addTemporalMarker(()->{})
                                .lineToConstantHeading(new Vector2d(51, 60))
                                .build()
                );


                /*.lineToLinearHeading(new Pose2d(-40-9/Math.sqrt(2), 24+(9/Math.sqrt(2)),Math.toRadians(315)))
                                .addTemporalMarker(()->{})
                                .setTangent(Math.PI+1.3613902)
                                .splineToLinearHeading(new Pose2d(-16, 6, Math.toRadians(70)), Math.toRadians(0))
                                .lineToLinearHeading(new Pose2d(20, 6, Math.toRadians(110)))
                                .setTangent(Math.toRadians(0))
                                .splineToLinearHeading(new Pose2d(51, 28, Math.toRadians(180)), Math.toRadians(90))
                                .addTemporalMarker(()->{})
                                .lineToConstantHeading(new Vector2d(51, 60))
                                .build() */

                /*.lineToLinearHeading(new Pose2d(9, 45, Math.toRadians(225)))
                .addTemporalMarker(()->{})
                .lineToLinearHeading(new Pose2d(51, 40, Math.toRadians(180)))
                .addTemporalMarker(()->{})
                .lineToConstantHeading(new Vector2d(51, 60))
                .build()
                */

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}