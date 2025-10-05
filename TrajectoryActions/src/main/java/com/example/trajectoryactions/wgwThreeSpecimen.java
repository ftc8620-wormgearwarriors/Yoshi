package com.example.trajectoryactions;


import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.example.trajectoryactions.SimConfig.Drive;


public class wgwThreeSpecimen extends wgwABCommon{


    public wgwThreeSpecimen(Drive d) {
        super(d);
    }

    public Action toSpikeMarkClose(Drive drive, Pose2d startPos, FieldSide fieldSide) {
        TrajectoryActionBuilder trajActBuilder = drive.actionBuilder(startPos);
        int dir = (fieldSide == FieldSide.BLUE) ? 1 : -1;
        return (trajActBuilder
                .setTangent(Math.toRadians(90 * dir))
                .splineToConstantHeading(new Vector2d(-30 * dir,35 * dir), Math.toRadians(225 * dir))
                .splineToConstantHeading(new Vector2d(-43 * dir,12 * dir), Math.toRadians(180 * dir))
//                .setTangent(Math.toRadians(135 * dir))
                .splineToConstantHeading(new Vector2d(-48 * dir,22 * dir), Math.toRadians(90 * dir))
                .splineToConstantHeading(new Vector2d(-48 * dir,47 * dir), Math.toRadians(90 * dir))



                .build());
    }

    public Action toSpikeMarkMiddle(Drive drive, Pose2d startPos, FieldSide fieldSide) {
        TrajectoryActionBuilder trajActBuilder = drive.actionBuilder(startPos);
        int dir = (fieldSide == FieldSide.BLUE) ? 1 : -1;
        return (trajActBuilder
                .setTangent(Math.toRadians(135 * dir))
                .splineToLinearHeading(new Pose2d(-34 * dir,38 * dir, Math.toRadians(270 * dir)), Math.toRadians(225 * dir))
                .splineToLinearHeading(new Pose2d(-45 * dir,12 * dir, Math.toRadians(270 * dir)), Math.toRadians(225 * dir))
                .splineToLinearHeading(new Pose2d(-59 * dir,16 * dir, Math.toRadians(270 * dir)), Math.toRadians(90 * dir))
//                .setTangent(Math.toRadians(135 * dir))
                .splineToLinearHeading(new Pose2d(-59 * dir,22 * dir, Math.toRadians(270 * dir)), Math.toRadians(90 * dir))
                .splineToLinearHeading(new Pose2d(-59 * dir,47 * dir, Math.toRadians(270 * dir)), Math.toRadians(90 * dir))



                .build());
    }

    public Action toRiggingFromObservation(Drive drive, Pose2d startPos, FieldSide fieldSide, double specimenXValue) {

        TrajectoryActionBuilder trajActBuilder = drive.actionBuilder(startPos);
        int dir = (fieldSide == FieldSide.BLUE) ? 1 : -1;
        return (trajActBuilder
//                .setReversed(false)
                .setTangent(Math.toRadians(0 * dir))
                .splineToConstantHeading(new Vector2d(specimenXValue * dir, 28.5 * dir), Math.toRadians(270 * dir))
                .build());
    }

    public Action toPickUpSpecimen(Drive drive, Pose2d startPos, FieldSide fieldSide) {
        TrajectoryActionBuilder trajActBuilder = drive.actionBuilder(startPos);
        int dir = (fieldSide == FieldSide.BLUE) ? 1 : -1;
        return (trajActBuilder
                .lineToY(54 * dir)
                .build());
    }



    public SequentialAction threeSpecimen(Drive drive, Action specimenToHighBarAction, Action wristToHighBarAction,
                                            Action specimenSlidesToHighBarAction, Action deliverSample, Action specimenPickUp,
                                            Action pickupSpecimen, Action specimenAfterPickup, Action slidesToPickup)
    {
        SequentialAction retVal;
        drive.setPose(wgwABCommon.startPosA2);
        Action driveToRiggingAction    = toRigging(drive,startPosA2,FieldSide.BLUE, 0);

        Action backUpFromRiggingAction = backUpFromRigging(drive, drive.findEndPos(driveToRiggingAction),FieldSide.BLUE, 0);

        Action driveToSpikeMarkCloseAction = toSpikeMarkClose(drive, drive.findEndPos(backUpFromRiggingAction),FieldSide.BLUE);

        Action driveToRiggingAction2 = toRiggingFromObservation(drive, drive.findEndPos(driveToSpikeMarkCloseAction),FieldSide.BLUE, -1);

        Action backUpFromRiggingAction2 = backUpFromRigging(drive, drive.findEndPos(driveToRiggingAction2),FieldSide.BLUE, -1);

        Action driveToSpikeMarkMiddle = toSpikeMarkMiddle(drive, drive.findEndPos(backUpFromRiggingAction2),FieldSide.BLUE);

        Action driveToRiggingAction3 = toRiggingFromObservation(drive, drive.findEndPos(driveToSpikeMarkMiddle),FieldSide.BLUE, -2);

        Action backUpFromRiggingAction3 = backUpFromRigging(drive, drive.findEndPos(driveToRiggingAction3),FieldSide.BLUE, -2);

        Action toParkAction = toPark(drive, drive.findEndPos(backUpFromRiggingAction3),FieldSide.BLUE);

        // Action driveToParkAction = toPark(drive, drive.findEndPos(driveToRiggingAction), FieldSide.BLUE);

        //Action driveToSpikeMarFarAction = toSpikeMarkFarFromRigging(drive, drive.findEndPos(driveToParkAction), FieldSide.BLUE);


        retVal = new SequentialAction(

                //parallel to drive to rigging, move arm up, and move wrist to deliver specimen position
                new ParallelAction(
                        driveToRiggingAction,
                        specimenToHighBarAction,
                        wristToHighBarAction),

                //move lift up
                specimenSlidesToHighBarAction,

                //robot back away from rigging
                backUpFromRiggingAction,

                // open claw to deliver specimen
               // deliverSample,

                // parallel to drive toward next specimen and move lift, arm, and wrist to pickup pos
                new ParallelAction(deliverSample, driveToSpikeMarkCloseAction, specimenPickUp),

                // close claw to pick up
                pickupSpecimen,


                // move arm to avoid hitting wall after grabbing specimen
                specimenToHighBarAction,

                // parallel to drive back to rigging and move wrist for pickup
                new ParallelAction(driveToRiggingAction2, wristToHighBarAction),

                // move lift up
                specimenSlidesToHighBarAction,

                // robot back away from rigging
                backUpFromRiggingAction2,

                //drive to next specimen
                new ParallelAction(deliverSample, specimenPickUp, driveToSpikeMarkMiddle),

                pickupSpecimen,

                // move arm to avoid hitting wall after grabbing specimen
                specimenToHighBarAction,

                // parallel to drive back to rigging and move slides down for pickup
                new ParallelAction(driveToRiggingAction3, wristToHighBarAction),

                // move lift up
                specimenSlidesToHighBarAction,

                // robot back away from rigging
                backUpFromRiggingAction3,

                // open claw to deliver specimen
                new ParallelAction(deliverSample, toParkAction, slidesToPickup)


        ) ;

        return retVal;
    }

}


