package com.example.trajectoryactions;


import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.example.trajectoryactions.SimConfig.Drive;


public class wgwBasketPreload extends wgwABCommon{


    public wgwBasketPreload(Drive d) {
        super(d);
    }



    public SequentialAction basketPreload(Drive drive, Action deliverSample, Action armToHighBasket,
                                          Action slideToHighBasket, Action armToSafety, Action slideToPickUp,
                                          Action wristToAscent1, Action armToAscent1)
    {
        SequentialAction retVal;

        //TODO accommodate a blue and red side; figure out why set pose isn't working
        drive.setPose(wgwABCommon.startPosA5);
        Action driveToBasketAction = toBasket(drive,startPosA5,FieldSide.BLUE);

        Action driveToAscentAction = toAscent(drive, drive.findEndPos(driveToBasketAction), FieldSide.BLUE);

        retVal = new SequentialAction(
            new ParallelAction(
                    driveToBasketAction,
                    armToSafety,
                    slideToHighBasket),
            armToHighBasket,
           // deliverSample,
            armToSafety,
            slideToPickUp,
            driveToAscentAction,
            new ParallelAction(
                    wristToAscent1,
                    armToAscent1)


        );

        return retVal;
    }

}


