package com.example.trajectoryactions;


import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.example.trajectoryactions.SimConfig.Drive;


public class wgwSampleAndSpecimen extends wgwABCommon {


    public wgwSampleAndSpecimen(Drive d) {
        super(d);
    }

    /**
     * drives from basket to the neutral spike marks on the blue side
    */


    public Action toTank(Drive drive, Pose2d startPos, FieldSide fieldSide) {
        TrajectoryActionBuilder trajActBuilder = drive.actionBuilder(startPos);
        int dir = (fieldSide == FieldSide.BLUE) ? 1 : -1;
        return (trajActBuilder
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(-3  * dir, 29 * dir, Math.toRadians(90 * dir)), Math.toRadians(270 * dir))
                .build());
    }

    public Action toBasket(Drive drive, Pose2d startPos, FieldSide fieldSide)  {
        TrajectoryActionBuilder trajActBuilder = drive.actionBuilder(startPos);
        int dir = (fieldSide == FieldSide.BLUE) ? 1 : -1;
        return (trajActBuilder
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(53  * dir, 59 * dir, Math.toRadians(180 * dir)), Math.toRadians(0))
                .build());

    }




    public SequentialAction sampleAndSpecimen(Drive drive, Action deliverSample, Action deliverSpecimen, Action pickUpSample, Action armToSpecimen)
    {
        SequentialAction retVal;
        Action driveToRiggingAction    = toRigging(drive,startPosA2,FieldSide.BLUE, 0);
        Action driveToTankAction       = toTank(drive, drive.findEndPos(driveToRiggingAction), FieldSide.BLUE);
        Action driveToBasketAction     = toBasket(drive, drive.findEndPos(driveToTankAction), FieldSide.BLUE);
        Action driveToParkAction = toPark(drive, drive.findEndPos(driveToBasketAction), FieldSide.BLUE);


        retVal = new SequentialAction(
            driveToRiggingAction,
            deliverSpecimen,
            driveToTankAction,
            pickUpSample,
            driveToBasketAction,
            deliverSample,
            driveToParkAction
        ) ;

        return retVal;
    }

}


