package com.example.trajectoryactions.SampleTrajectories;

// this example demonstrates how methods can be used (and re-used) to create sequential actions
// in previous years we have used a method to create the path for each of the randomized targets
// and then reused those same paths at all of our robot start positions.
// Design the trajectories to be resuable when possible saves on code tweatking and confusion.

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.example.trajectoryactions.SimConfig.Drive;
import com.example.trajectoryactions.SimConfig.SimViewStartMarker;

public class AutoSpecimens extends commonTrajectories {
    Drive drive;
    public AutoSpecimens(Drive d) {

        super(d);
        drive = d;
    }

    Pose2d endPos = new Pose2d(48,-52, Math.PI*3/2);

    SequentialAction deliverSpecimen(Pose2d start, int specimen){

        TrajectoryActionBuilder toSubmersible = drive.actionBuilder(start)
                .setReversed(true)
                .splineToLinearHeading(transform(3*specimen ,-34, Math.PI*3/2), xformHeading(Math.PI/2));

        TrajectoryActionBuilder toSamples = toSubmersible.endTrajectory().fresh()
                .setReversed(false)
                .splineToSplineHeading(transform(30 ,-36, 0), xformHeading(0))
                .splineToLinearHeading(transform(40+10*specimen ,-26, 0), xformHeading(0));

        TrajectoryActionBuilder toObservationZone1 = toSamples.endTrajectory().fresh()
                .setReversed(true)
                .splineToLinearHeading(transform(endPos), xformHeading(0));

        TrajectoryActionBuilder toObservationZone2 = toObservationZone1.endTrajectory().fresh()
                .setReversed(false)
                .splineToLinearHeading(transform(48,-48, Math.PI*3/2), xformHeading(Math.PI*3/2))
                //.waitSeconds(2)
                .splineToLinearHeading(transform(endPos), xformHeading(Math.PI*1/2));

        return new SequentialAction(
                new ParallelAction(
                    toSubmersible.build(),
                    actionParameters.liftUp),
                actionParameters.deliverSpecimen,
                new ParallelAction(
                        toSamples.build(),
                        actionParameters.liftDown),
                actionParameters.collectSample,
                toObservationZone1.build(),
                actionParameters.deliverSample,
                toObservationZone2.build(),
                actionParameters.CollectSpecimen
        );
    }


    public SequentialAction allSpecimens() {

        drive.setPose(transform(startPosF4));

        Action d1 = deliverSpecimen(drive.getPose(), 0);
        Action d2 = deliverSpecimen(transform(endPos), 1);
        Action d3 = deliverSpecimen(transform(endPos), 2);

        Action park = drive.actionBuilder(transform(endPos))
                .setReversed(true)
                .splineToSplineHeading(transform(24 ,-12, Math.PI*3/2), xformHeading(Math.PI)).build();

        return new SequentialAction(
                d1,     // deliver preload, pick up 1st sample and take to  human to make specimen
                d2,     // deliver 2nd specimena dn pick up sample and have human make specimen
//                new SimViewStartMarker(),
                park    // no time to deliver, go park.
        );
    }
    public  SequentialAction actionTest(){
        return new SequentialAction(
                new SimTimedAction("A 10 second", 5.0),
                new SimViewStartMarker(),
                new SimTimedAction("B 10 second", 5.0)

//                drive.actionBuilder(new Pose2d(0,0,0)).turn(Math.PI*6).build()
//                actionParameters.collectSample,
//                actionParameters.liftUp,
//                actionParameters.deliverSample,
//                actionParameters.liftDown,
//                actionParameters.collectSample,
//                actionParameters.liftUp,
//                actionParameters.deliverSample,
//                actionParameters.liftDown
        );
    }

}
