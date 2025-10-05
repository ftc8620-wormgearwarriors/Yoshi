package com.example.trajectoryactions.Paths;

// this example demonstrates how methods can be used (and re-used) to create sequential actions
// in previous years we have used a method to create the path for each of the randomized targets
// and then reused those same paths at all of our robot start positions.
// Design the trajectories to be resuable when possible saves on code tweatking and confusion.

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.example.trajectoryactions.SampleTrajectories.commonTrajectories;
import com.example.trajectoryactions.SimConfig.Drive;

public class ArtifactClosePath extends commonTrajectories {
    Drive drive;
    public ArtifactClosePath(Drive d) {

        super(d);
        drive = d;
    }

    Pose2d endPos = new Pose2d(48,-52, Math.PI*3/2);

    SequentialAction artifactClosePath(Pose2d start){

        TrajectoryActionBuilder driveOffLaunchLine = drive.actionBuilder(start)
                .setTangent(Math.PI*3/2)
                .splineToLinearHeading(transform(-48 ,0.7, Math.PI*1/2), xformHeading(Math.PI*3/2));

        return new SequentialAction(
                driveOffLaunchLine.build()
        );
    }

    public SequentialAction allArtifacts() {
        drive.setPose(transform(startPosA6));
        return new SequentialAction(artifactClosePath(startPosA6));
        //.waitSeconds(2)
    }

}
