package com.example.trajectoryactions;


import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.example.trajectoryactions.SimConfig.Drive;

import java.util.Objects;


public class wgwFollowTheLine extends wgwABCommon{


    public wgwFollowTheLine(Drive d) {
        super(d);
    }

public Action straightLine(Drive drive, Pose2d startPos, double endPos) {
    TrajectoryActionBuilder trajActBuilder = drive.actionBuilder(startPos);
    return (trajActBuilder
            .lineToX(endPos)
            .build());
}

    public Action turn(Drive drive, Pose2d startPos, String direction, double angle) {
        TrajectoryActionBuilder trajActBuilder = drive.actionBuilder(startPos);
        int dir = (Objects.equals(direction, "LEFT")) ? 1 : -1;

        return (trajActBuilder
                .turn(Math.toRadians(angle*dir))
                .build());
    }

    public SequentialAction followTheLine(Drive drive)
    {
        SequentialAction retVal;

        drive.setPose(wgwABCommon.startCenterField);

        Action straightLine1 = straightLine(drive, startCenterField, 20);

        Action turn1 = turn(drive, drive.findEndPos(straightLine1), "RIGHT", 60);

        Action straightLine2 = straightLine(drive, drive.findEndPos(turn1), 25);

        retVal = new SequentialAction(

                straightLine1,

                turn1,

                straightLine2


        );

        return retVal;
    }


}

