package com.example.trajectoryactions.SampleTrajectories;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.example.trajectoryactions.SimConfig.Drive;

public class YellowSamples extends commonTrajectories {
    Drive drive;
    public YellowSamples(Drive d) {
        super(d);
        drive = d;
    }

    public SequentialAction allYellows () {

        Pose2d basketPos = new Pose2d(-56,-56,Math.PI/4);

        drive.setPose(transform(startPosF2));

        // *** tried to use fresh() for starting point of next trajectory but did not work.
        TrajectoryActionBuilder toFirstYellow = drive.actionBuilder(drive.getPose())
                .setReversed(false)
                .splineToSplineHeading(transform(-43 ,-26, Math.PI), xformHeading(Math.PI));

        TrajectoryActionBuilder firstYellow2Basket = toFirstYellow.endTrajectory().fresh()
                .setReversed(true)
                .splineToLinearHeading(transform(basketPos), xformHeading(Math.PI * 5/4));

        TrajectoryActionBuilder toSecondYellow = firstYellow2Basket.endTrajectory().fresh()
                .setReversed(false)
                .splineToSplineHeading(transform(-53 ,-26, Math.PI), xformHeading(Math.PI));

        TrajectoryActionBuilder secondYellow2Basket = toSecondYellow.endTrajectory().fresh()
                .setReversed(true)
                .splineToLinearHeading(transform(basketPos), xformHeading(Math.PI * 5/4));

        TrajectoryActionBuilder toThirdYellow = secondYellow2Basket.endTrajectory().fresh()
                .setReversed(false)
                .splineToSplineHeading(transform(-63 ,-26, Math.PI), xformHeading(Math.PI));

        TrajectoryActionBuilder thirdYellow2Basket = toThirdYellow.endTrajectory().fresh()
                .setReversed(true)
                .splineToLinearHeading(transform(basketPos), xformHeading(Math.PI * 5/4));

        TrajectoryActionBuilder toFourthYellow = thirdYellow2Basket.endTrajectory().fresh()
                .setReversed(false)
                .splineToSplineHeading(transform(-24 ,-12, 0), xformHeading(0));

        TrajectoryActionBuilder fourthYellow2Basket = toFourthYellow.endTrajectory().fresh()
                .setReversed(true)
                .splineToLinearHeading(transform(basketPos), xformHeading(Math.PI * 5/4));

        TrajectoryActionBuilder park = fourthYellow2Basket.endTrajectory().fresh()
                .setReversed(false)
                .splineToSplineHeading(transform(-24 ,-12, 0), xformHeading(0));

        return new SequentialAction(
                toFirstYellow.build(),
                actionParameters.collectSample,
                new ParallelAction(  // drive to basket and lift up at same time
                        firstYellow2Basket.build(),
                        actionParameters.liftUp),
                actionParameters.deliverSample,
                new ParallelAction(  // drive and put lift down at same time
                    toSecondYellow.build(),
                    actionParameters.liftDown),
                actionParameters.collectSample,
                new ParallelAction(
                    secondYellow2Basket.build(),
                    actionParameters.liftUp),
                actionParameters.deliverSample,
                toThirdYellow.build(),
                actionParameters.collectSample,
                new ParallelAction(
                    thirdYellow2Basket.build(),
                    actionParameters.liftUp),
                actionParameters.deliverSample,
                toFourthYellow.build(),
                actionParameters.collectSample,
                new ParallelAction(
                    fourthYellow2Basket.build(),
                    actionParameters.liftUp),
                actionParameters.deliverSample,
                park.build()
        );
    }
    public SequentialAction allYellowsLong () {

        Pose2d basketPos = new Pose2d(-56,-56,Math.PI/4);

        drive.setPose(transform(startPosF2));

        return new SequentialAction(
            drive.actionBuilder(drive.getPose())
                .splineToSplineHeading(transform(-43 ,-26, Math.PI), xformHeading(Math.PI))
                .stopAndAdd(actionParameters.collectSample)
                .setReversed(true)
                .splineToLinearHeading(transform(basketPos), xformHeading(Math.PI * 5/4))
                .stopAndAdd(new SequentialAction(
                        actionParameters.liftUp,
                        actionParameters.deliverSample,
                        actionParameters.liftDown))
                .splineToSplineHeading(transform(-53 ,-26, Math.PI), xformHeading(Math.PI))
                .stopAndAdd(actionParameters.collectSample)
                .setReversed(true)
                .splineToLinearHeading(transform(basketPos), xformHeading(Math.PI * 5/4))
                .stopAndAdd(new SequentialAction(
                        actionParameters.liftUp,
                        actionParameters.deliverSample,
                        actionParameters.liftDown))
                .splineToSplineHeading(transform(-63 ,-26, Math.PI), xformHeading(Math.PI))
                .stopAndAdd(actionParameters.collectSample)
                .setReversed(true)
                .splineToLinearHeading(transform(basketPos), xformHeading(Math.PI * 5/4))
                .stopAndAdd(new SequentialAction(
                        actionParameters.liftUp,
                        actionParameters.deliverSample,
                        actionParameters.liftDown))
                .splineToSplineHeading(transform(-24 ,-12, 0), xformHeading(0))
                .stopAndAdd(actionParameters.collectSample)
                .setReversed(true)
                .splineToLinearHeading(transform(basketPos), xformHeading(Math.PI * 5/4))
                .stopAndAdd(new SequentialAction(
                        actionParameters.liftUp,
                        actionParameters.deliverSample,
                        actionParameters.liftDown))
                .splineToSplineHeading(transform(-24 ,-12, 0), xformHeading(0)).build()
            );

    }
}
