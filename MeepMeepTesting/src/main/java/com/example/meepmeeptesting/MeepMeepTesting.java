package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 18)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12, 63, Math.toRadians(270)))
                                .lineToConstantHeading(new Vector2d (12,34))
                                .addTemporalMarker(0, () -> {})
                                .lineToSplineHeading(new Pose2d(50, 34, 0))
                                .addTemporalMarker(0, () -> {})
                                .lineToConstantHeading(new Vector2d(50, 56))
                                .build()
                );

        /*
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 18)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12, 63, Math.toRadians(270)))
                                .splineToSplineHeading(new Pose2d (0,34, Math.PI), Math.PI)
                                .addTemporalMarker(0, () -> {})
                                .lineToConstantHeading(new Vector2d(50, 28))
                                .addTemporalMarker(0, () -> {})
                                .lineToConstantHeading(new Vector2d(50, 56))
                                .build()
                );
        */
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}