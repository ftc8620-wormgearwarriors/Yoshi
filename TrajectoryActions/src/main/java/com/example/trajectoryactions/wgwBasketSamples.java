package com.example.trajectoryactions;


import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.example.trajectoryactions.SimConfig.Drive;


public class wgwBasketSamples extends wgwABCommon{


    public wgwBasketSamples(Drive d) {
        super(d);
    }



//    public Action toBasket(Drive drive, Pose2d startPos, FieldSide fieldSide) {
//        TrajectoryActionBuilder trajActBuilder = drive.actionBuilder(startPos);
//        int dir = (fieldSide == FieldSide.BLUE) ? 1 : -1;
//        return (trajActBuilder
//                .setTangent(0)
//                .splineToSplineHeading(new Pose2d(52 * dir,59 * dir, Math.toRadians(180 * dir)), Math.toRadians(0 * dir))
//                .build());
//    }



    /**
     * Far, Middle, and Close for spike marks relate to the distance from the wall
    */
    public Action toSpikeMarkFar(Drive drive, Pose2d startPos, FieldSide fieldSide) {
        TrajectoryActionBuilder trajActBuilder = drive.actionBuilder(startPos);
        int dir = (fieldSide == FieldSide.BLUE) ? 1 : -1;
        return (trajActBuilder
                .setTangent(Math.toRadians(225 * dir))
                .splineToSplineHeading(new Pose2d(48 * dir, 44.5 * dir, Math.toRadians(90 * dir)), Math.toRadians(270 * dir))
                .build());
    }



    public Action toSpikeMarkMiddle(Drive drive, Pose2d startPos, FieldSide fieldSide) {
        TrajectoryActionBuilder trajActBuilder = drive.actionBuilder(startPos);
        int dir = (fieldSide == FieldSide.BLUE) ? 1 : -1;
        return (trajActBuilder
                .setTangent(Math.toRadians(270 * dir))
                .splineToSplineHeading(new Pose2d(59 * dir, 44 * dir, Math.toRadians(90 * dir)), Math.toRadians(270 * dir))
                .build());
    }

    public Action toBasketSpikeMarks(Drive drive, Pose2d startPos, FieldSide fieldSide) {
        TrajectoryActionBuilder trajActBuilder = drive.actionBuilder(startPos);
        int dir = (fieldSide == FieldSide.BLUE) ? 1 : -1;
        return (trajActBuilder
                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(basketX * dir,basketY * dir, Math.toRadians(225 * dir)), Math.toRadians(90 * dir))
                .build());
    }

    public Action toSpikeMarkClose1(Drive drive, Pose2d startPos, FieldSide fieldSide) {
        TrajectoryActionBuilder trajActBuilder = drive.actionBuilder(startPos);
        int dir = (fieldSide == FieldSide.BLUE) ? 1 : -1;
        return (trajActBuilder
                .setTangent(225 * dir)
                .splineToSplineHeading(new Pose2d(55 * dir, 48 * dir, Math.toRadians(125 * dir)), Math.toRadians(315 * dir))
                .build());
    }

    public Action toSpikeMarkClose2(Drive drive, Pose2d startPos, FieldSide fieldSide) {
        TrajectoryActionBuilder trajActBuilder = drive.actionBuilder(startPos);
        int dir = (fieldSide == FieldSide.BLUE) ? 1 : -1;
        return (trajActBuilder
                .setTangent(Math.toRadians(315 * dir))
                //.splineToSplineHeading(new Pose2d(60 * dir, 42 * dir, Math.toRadians(125 * dir)), Math.toRadians(315 * dir))
                .splineToConstantHeading(new Vector2d(58 * dir, 41.5 * dir), Math.toRadians(312 * dir))
                .build());
    }

        public Action toBasketSpikeMarkClose(Drive drive, Pose2d startPos, FieldSide fieldSide) {
            TrajectoryActionBuilder trajActBuilder = drive.actionBuilder(startPos);
            int dir = (fieldSide == FieldSide.BLUE) ? 1 : -1;
            return (trajActBuilder
                    .setTangent(Math.toRadians(135))
                    .splineToSplineHeading(new Pose2d(basketX * dir, basketY * dir, Math.toRadians(225 * dir)), Math.toRadians(135 * dir))
                    .build());


    }

 
    public SequentialAction basketSamples(Drive drive, Action deliverSample, Action pickUpSample, Action armToPickUp,
                                          Action armToHighBasket, Action slideToHighBasket, Action armToAscent1, Action slideToPickUp,
                                          Action armToSafety, Action armWhenDriving)
    {
        SequentialAction retVal;
        drive.setPose(wgwABCommon.startPosA5);
        Action driveToBasketAction    = toBasket(drive,startPosA5,FieldSide.BLUE);

        Action driveToSpikeMarkFarAction = toSpikeMarkFar(drive, drive.findEndPos(driveToBasketAction), FieldSide.BLUE);

        Action driveToBasketAction2   = toBasketSpikeMarks(drive, drive.findEndPos(driveToSpikeMarkFarAction),FieldSide.BLUE);

        Action driveToAscent          = toAscent(drive, drive.findEndPos(driveToBasketAction2), FieldSide.BLUE);

        Action driveToSpikeMarkMiddleAction = toSpikeMarkMiddle(drive, drive.findEndPos(driveToBasketAction2), FieldSide.BLUE);

        Action driveToBasketAction3   = toBasketSpikeMarks(drive, drive.findEndPos(driveToSpikeMarkMiddleAction),FieldSide.BLUE);

        Action driveToSpikeMarkClose1Action = toSpikeMarkClose1(drive, drive.findEndPos(driveToBasketAction3), FieldSide.BLUE);

        Action driveToSpikeMarkClose2Action = toSpikeMarkClose2(drive, drive.findEndPos(driveToSpikeMarkClose1Action), FieldSide.BLUE);

        Action driveToBasketSpikeMarkClose   = toBasketSpikeMarkClose(drive, drive.findEndPos(driveToSpikeMarkClose2Action),FieldSide.BLUE);


        retVal = new SequentialAction(

            // deliver preload and pick up first sample
            // drive toward basket, move arm to safety pos for slide to move up in ||
            new ParallelAction(driveToBasketAction, armToSafety, slideToHighBasket),
            // move the arm to the high basket delivery position
            armToHighBasket,
            // open the claw to deliver
            deliverSample,
            //move the arm back to the safety pos
            armToSafety,
            // move the slide down and drive to far spike mark in ||
            new ParallelAction(slideToPickUp,driveToSpikeMarkFarAction),
            // move the arm to the pick up pos
            armToPickUp,
            // close the claw to pick up far sample
            pickUpSample,

            // deliver far sample and pick up mid sample
            // drive back to the basket, move arm to safety for slide to go up in ||
            new ParallelAction(driveToBasketAction2, armToSafety, slideToHighBasket),
            // move the arm to the high basket delivery pos
            armToHighBasket,
            // open the claw to deliver
            deliverSample,
            // move the arm back to the safety pos
            armToSafety,
            // drive to mid spike and move slide down in ||
            new ParallelAction(driveToSpikeMarkMiddleAction, slideToPickUp),
            // move the arm to pick up pos
            armToPickUp,
            // close the claw to pick up mid sample
            pickUpSample,

            // deliver mid sample and pick up close sample
            // drive back to the basket, move arm to safety for slide to go up in ||
            new ParallelAction(armToSafety, driveToBasketAction3, slideToHighBasket),
            // move arm to high basket delivery
            armToHighBasket,
            // open the claw to deliver
            deliverSample,
            // move the arm back to the safety pos
            armToSafety,
            // drive to intermediate pos before close spike, slide down in ||
            new ParallelAction(driveToSpikeMarkClose1Action, slideToPickUp),
            // move the arm to driving pos for close spike

            new ParallelAction(armWhenDriving, driveToSpikeMarkClose2Action),
            // drive to actual pos for close spike

            // move the arm to pick up pos
            armToPickUp,
            // close the claw to pick up close sample
            pickUpSample,

            // deliver close sample and park
            // drive back to basket, arm to safety for slide to go up in ||
            new ParallelAction(driveToBasketSpikeMarkClose, armToSafety, slideToHighBasket),
            // move arm to high basket delivery
            armToHighBasket,
            // open claw to deliver
            deliverSample,
            // move arm back to safety pos
            armToSafety,
            // drive to ascent, slide down in ||
            new ParallelAction(slideToPickUp, driveToAscent),
            // move arm to score ascent parking
            armToAscent1
        ) ;

        return retVal;
    }

}


