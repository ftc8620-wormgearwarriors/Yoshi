package com.example.trajectoryactions;


import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.example.trajectoryactions.SimConfig.Drive;


public class wgwFourSpecimen extends wgwABCommon{


    public wgwFourSpecimen(Drive d) {
        super(d);
    }

    public Action toSpikeMarkClose(Drive drive, Pose2d startPos, FieldSide fieldSide) {
        TrajectoryActionBuilder trajActBuilder = drive.actionBuilder(startPos);
        int dir = (fieldSide == FieldSide.BLUE) ? 1 : -1;
        return (trajActBuilder
                .setTangent(Math.toRadians(135 * dir))
//                .splineToLinearHeading(new Pose2d(-30 * dir,35 * dir, Math.toRadians(270 * dir)), Math.toRadians(225 * dir))
//                .splineToLinearHeading(new Pose2d(-43 * dir,10 * dir, Math.toRadians(270 * dir)), Math.toRadians(180 * dir))
//                .splineToLinearHeading(new Pose2d(-48 * dir,22 * dir, Math.toRadians(270 * dir)), Math.toRadians(90 * dir))
//                .splineToLinearHeading(new Pose2d(-48 * dir,47 * dir, Math.toRadians(270 * dir)), Math.toRadians(90 * dir))
                .splineToConstantHeading(new Vector2d(-30 * dir,35 * dir), Math.toRadians(225 * dir))
                .splineToConstantHeading(new Vector2d(-43 * dir,10 * dir), Math.toRadians(180 * dir))
                .splineToConstantHeading(new Vector2d(-48 * dir,22 * dir), Math.toRadians(90 * dir))
                .splineToConstantHeading(new Vector2d(-48 * dir,47 * dir), Math.toRadians(90 * dir))



                .build());
    }

    public Action toSpikeMarkMiddle(Drive drive, Pose2d startPos, FieldSide fieldSide) {
        TrajectoryActionBuilder trajActBuilder = drive.actionBuilder(startPos);
        int dir = (fieldSide == FieldSide.BLUE) ? 1 : -1;
        return (trajActBuilder
                .setTangent(Math.toRadians(135 * dir))
//                .splineToLinearHeading(new Pose2d(-34 * dir,38 * dir, Math.toRadians(270 * dir)), Math.toRadians(225 * dir))
//                .splineToLinearHeading(new Pose2d(-45 * dir,10 * dir, Math.toRadians(270 * dir)), Math.toRadians(225 * dir))
//                .splineToLinearHeading(new Pose2d(-56 * dir,16 * dir, Math.toRadians(270 * dir)), Math.toRadians(90 * dir))
//                .splineToLinearHeading(new Pose2d(-56 * dir,22 * dir, Math.toRadians(270 * dir)), Math.toRadians(90 * dir))
//                .splineToLinearHeading(new Pose2d(-56 * dir,47 * dir, Math.toRadians(270 * dir)), Math.toRadians(90 * dir))
                .splineToConstantHeading(new Vector2d(-34 * dir,38 * dir), Math.toRadians(225 * dir))
                .splineToConstantHeading(new Vector2d(-45 * dir,10 * dir), Math.toRadians(225 * dir))
                .splineToConstantHeading(new Vector2d(-56 * dir,16 * dir), Math.toRadians(90 * dir))
                .splineToConstantHeading(new Vector2d(-56 * dir,22 * dir), Math.toRadians(90 * dir))
                .splineToConstantHeading(new Vector2d(-56 * dir,47.5 * dir), Math.toRadians(90 * dir))
                .build());
    }

    public Action toRiggingFromObservation(Drive drive, Pose2d startPos, FieldSide fieldSide, double specimenXValue) {

        TrajectoryActionBuilder trajActBuilder = drive.actionBuilder(startPos);
        int dir = (fieldSide == FieldSide.BLUE) ? 1 : -1;
        return (trajActBuilder
                .splineToConstantHeading(new Vector2d(specimenXValue * dir, 29 * dir), Math.toRadians(270 * dir))
                .build());
    }

    public Action toObservation(Drive drive, Pose2d startPos, FieldSide fieldSide) {

        TrajectoryActionBuilder trajActBuilder = drive.actionBuilder(startPos);
        int dir = (fieldSide == FieldSide.BLUE) ? 1 : -1;
        return (trajActBuilder
                .setReversed(true)
//                .splineToConstantHeading(new Vector2d(-20 * dir, 40 * dir), Math.toRadians(180 * dir))
                .splineToLinearHeading(new Pose2d(-40 * dir,48 * dir, Math.toRadians(274)), Math.toRadians(90 * dir))

                .build());
    }

    public SequentialAction fourSpecimen(Drive drive, Action specimenToHighBarAction, Action wristToHighBarAction,
                                          Action specimenSlidesToHighBarAction, Action deliverSample, Action specimenPickUp,
                                          Action pickupSpecimen, Action specimenAfterPickup, Action slidesToPickup)
    {
        SequentialAction retVal;
        drive.setPose(wgwABCommon.startPosA2);
        Action driveToRiggingAction    = toRigging(drive,startPosA2,FieldSide.BLUE, 2);

        Action backUpFromRiggingAction = backUpFromRigging(drive, drive.findEndPos(driveToRiggingAction),FieldSide.BLUE, 2);

        Action driveToSpikeMarkCloseAction = toSpikeMarkClose(drive, drive.findEndPos(backUpFromRiggingAction),FieldSide.BLUE);

        Action driveToRiggingAction2 = toRiggingFromObservation(drive, drive.findEndPos(driveToSpikeMarkCloseAction),FieldSide.BLUE,0);

        Action backUpFromRiggingAction2 = backUpFromRigging(drive, drive.findEndPos(driveToRiggingAction2),FieldSide.BLUE,0);

        Action driveToSpikeMarkMiddle = toSpikeMarkMiddle(drive, drive.findEndPos(backUpFromRiggingAction2),FieldSide.BLUE);

        Action driveToRiggingAction3 = toRiggingFromObservation(drive, drive.findEndPos(driveToSpikeMarkMiddle),FieldSide.BLUE,-2);

        Action backUpFromRiggingAction3 = backUpFromRigging(drive, drive.findEndPos(driveToRiggingAction3),FieldSide.BLUE,-2);

        Action driveToObservation = toObservation(drive, drive.findEndPos(backUpFromRiggingAction3),FieldSide.BLUE);

        Action driveToRiggingAction4 = toRiggingFromObservation(drive, drive.findEndPos(driveToObservation),FieldSide.BLUE,-4);

        Action backUpFromRiggingAction4 = backUpFromRigging(drive, drive.findEndPos(driveToRiggingAction4),FieldSide.BLUE,-4);

        Action toParkAction = toPark(drive, drive.findEndPos(backUpFromRiggingAction4),FieldSide.BLUE);

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
//                deliverSample,

                // parallel to drive toward next specimen and move lift, arm, and wrist to pickup pos
                new ParallelAction(deliverSample,driveToSpikeMarkCloseAction, specimenPickUp),

                // close claw to pick up
                pickupSpecimen,

                // move arm for delivering to high bar
                specimenToHighBarAction,

                // parallel to drive back to rigging and move wrist for pickup
                new ParallelAction(wristToHighBarAction, driveToRiggingAction2),

                // move lift up
                specimenSlidesToHighBarAction,

                // robot back away from rigging
                backUpFromRiggingAction2,

                // open claw to deliver specimen
                deliverSample,

                //drive to next specimen
                new ParallelAction(driveToSpikeMarkMiddle, specimenPickUp),

                // move slides up after grabbing specimen
                pickupSpecimen,

                // move arm up to delivery at high bar
                specimenToHighBarAction,

                // parallel to drive back to rigging and move slides down for pickup
                new ParallelAction( wristToHighBarAction, driveToRiggingAction3),

                // move lift up
                specimenSlidesToHighBarAction,

                // robot back away from rigging
                backUpFromRiggingAction3,

                // open claw to deliver specimen, go back to observation, arm and slides for pickup
                new ParallelAction(deliverSample, driveToObservation, specimenPickUp),

                //Pick up specimen
                pickupSpecimen,

                // move arm up to delivery at high bar
                specimenToHighBarAction,

                // parallel to drive back to rigging and move slides down for pickup
                new ParallelAction(wristToHighBarAction, driveToRiggingAction4),

                // move lift up
                specimenSlidesToHighBarAction,

                // robot back away from rigging
                backUpFromRiggingAction4,

                // go to park, open claw, move slides down
                new ParallelAction(toParkAction, deliverSample, slidesToPickup)

        ) ;

        return retVal;
    }

}


