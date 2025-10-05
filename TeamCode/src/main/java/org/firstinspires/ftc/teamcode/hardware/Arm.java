package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import static com.acmerobotics.roadrunner.Actions.now;

import com.qualcomm.robotcore.util.RobotLog;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Arm {

    Robot ferb = null;

    // // declare hardware classes belonging to arm
    public Claw claw = null;
    public DcMotorEx rightSlideMotor = null;
    public DcMotorEx leftSlideMotor  = null;
    public DcMotor armMotor        = null;
    public DigitalChannel slideSensor  = null;
    public DigitalChannel armSensor  = null;

    // member variables
    private HardwareMap m_hwMap     = null;

    //
    int armInit                 = 0;
    int armSpecimenAuto         = 0; //used in specimen auto at the bar on the rigging

    // slide for delivering specimen at rigging
    int leftSlideSpecimenAuto   = 779;
    int rightSlideSpecimenAuto  = 771;

    // arm basket delivery positions
    int armLowBasketFront       = -630;
    int armHighBasketFront      = -530;
    int armHighBasketTele       = -530;

    // arm picking up sample
    int armPickup               = -1100;
    int autoArmPickup           = -1080;

    // slides after picking up specimen
    int leftSlideAfterSpecimen  =  602;
    int rightSlideAfterSpecimen = 637;

    // arm driving without dragging on ground
    int armDriving              = -730;
    int autoArmDriving          = -800;

    // arm picking up into submersible
    int armIntoSubmersible      = -550;

    // safety not to hit arm on things as slide lowers
    int armSafetyPosition       = -220;

    // low slides a little bit greater than zero to avoid motors fighting at bottom position
    int leftSlidePickup         = 15;
    int rightSlidePickup        = 15;

    int leftSlideHighBasket     = 3500; // Slide to deliver sample at the high basket
    int rightSlideHighBasket    = 3500;
    int leftAutoSlideHighBasket = 3500; // auto was 3675
    int rightAutoSlideHighBasket= 3500; // auto was 3675

    // arm chamber delivery positions
    int armHighBar              = -310; //-250
    int armLowBar               = -1045;

    int armSpecimenWall         = -903; // used in teleop for specimen off of wall
    int armSpecimenWallAuto     = -890; //used in specimen auto when picking up specimen from human player

    int armSpecimenGround       = -1034;

    int slideRightHighBar       = 998;
    int slideLeftHighBar        = 998;

    int armAscent1              = 0;

    boolean isArmSafe           = false;

    int leftSlideLowBasket      = 2078;
    int rightSlideLowBasket     = 2078;

    public boolean isArmHomedOnInit    = false;
//    boolean isSlideHomedOnInit   = false;
    // actual max is 4400
    // minimum 0

    public Arm(HardwareMap hardwareMap, Robot thisRobot){
       // instantiates claw for this class
        claw = new Claw(hardwareMap, thisRobot);
        // assigns the hardware map and makes arm motor for config
        m_hwMap = hardwareMap;
        leftSlideMotor = m_hwMap.get(DcMotorEx.class, "leftSlideMotor");
        rightSlideMotor = m_hwMap.get(DcMotorEx.class, "rightSlideMotor");
        armMotor = m_hwMap.get(DcMotor.class, "armMotor");
        armSensor = m_hwMap.get(DigitalChannel.class, "armSensor");


        //set slide motors reversed if needed
//        rightSlideMotor.setDirection(DcMotor.Direction.REVERSE);
        leftSlideMotor.setDirection(DcMotor.Direction.REVERSE);

        ferb = thisRobot;

    }

    public void init()
    {
        leftSlideMotor.setPower(0);
        rightSlideMotor.setPower(0);
        armMotor.setPower(0);
        leftSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setTargetPosition(0);
        leftSlideMotor.setTargetPosition(0);
        rightSlideMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void homing()
    {

        claw.start();

        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double startTime = now();

        while (!armIsHomed() && (now() - startTime) < 4.0)
        {
            armMotor.setPower(0.15);
            RobotLog.vv("WGW Arm Homing", "Arm is still Homing");
        }
         // it has been homed, so set variable to true
        isArmHomedOnInit = true;

        // if sensor is triggered, stop motor power and reset encoder to 0
        armMotor.setPower(0.0);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RobotLog.vv("WGW Arm Homing", "Arm is Homed");

        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);



//        while (!slideIsHomed())
//        {
//            leftSlideMotor.setPower(-0.25);
//            rightSlideMotor.setPower(-0.25);
//        }
//         // it has been homed, so set variable to true
//        isSlideHomedOnInit = true;
//
//        // if sensor is triggered, stop motor power and reset encoder to 0
//        leftSlideMotor.setPower(0.0);
//        leftSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightSlideMotor.setPower(0.0);
//        rightSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }
    public void slideHoming(){
        leftSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftSlideMotor.setPower(-0.1);
        rightSlideMotor.setPower(-0.1);

        double previousPosition = rightSlideMotor.getCurrentPosition();
        double deltaPos;
        double currentTime;
        double previousTime = now();
        double deltaTime;
        double distanceLimit = 25;
        double timeLimit = 0.2;
        boolean keepMoving = true;
        double currentPosition;
        double startTime = now();

        // get initial encoder reading
        // set slide power to go down slowly
        while (keepMoving)
        {
            currentPosition = rightSlideMotor.getCurrentPosition();
            currentTime = now();
            deltaTime = currentTime - previousTime;
            deltaPos = previousPosition - currentPosition;
            RobotLog.vv("WGW slide homing", "Current Position: " + currentPosition + ". Current Time: " + currentTime + ". Delta Time: " + deltaTime + ". Delta Position: "+ deltaPos);
            RobotLog.vv("WGW slide homing", "Left slide motor current: " +
                        leftSlideMotor.getCurrent(CurrentUnit.AMPS));
            RobotLog.vv("WGW slide homing", "Right slide motor current: " +
                        rightSlideMotor.getCurrent(CurrentUnit.AMPS));
            if (deltaTime > timeLimit ) {
                RobotLog.vv("WGW slide homing", "Delta Time is greater than time limit");
                RobotLog.vv("WGW slide homing", "Left slide motor current: " +
                        leftSlideMotor.getCurrent(CurrentUnit.AMPS));
                RobotLog.vv("WGW slide homing", "Right slide motor current: " +
                        rightSlideMotor.getCurrent(CurrentUnit.AMPS));
                if (deltaPos < distanceLimit) {
                    RobotLog.vv("WGW slide homing", "Delta Position is less than position limit. Is Homed");
                    keepMoving = false;
                }
                previousTime = currentTime;
                previousPosition = currentPosition;
            }

            // check encoder readings ever #
            // if the change in the encoder reading drops below a certain level then reset encoders
            if ((now() - startTime) > 8.0) {
                keepMoving = false;
                RobotLog.vv("WGW slide homing", "Max time exceeded");
            }

        }
        rightSlideMotor.setPower(0.0);
        rightSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlideMotor.setPower(0.0);
        leftSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


//    public void slideRunUp()
//    {
//        leftSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        leftSlideMotor.setPower(1.0);
//        rightSlideMotor.setPower(1.0);
//    }
//    public void slideStop()
//    {
//        leftSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        leftSlideMotor.setPower(0);
//        rightSlideMotor.setPower(0);
//    }
//    public void slideRunDown()
//    {
//        leftSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        leftSlideMotor.setPower(-0.3);
//        rightSlideMotor.setPower(-0.3);
//    }

    public void  slidePickup()
    {
        leftSlideMotor.setTargetPosition(leftSlidePickup);
        rightSlideMotor.setTargetPosition(rightSlidePickup);
        leftSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlideMotor.setPower(0.9);
        leftSlideMotor.setPower(0.9);
    }

    public void  slidesAfterSpecimenPickup()
    {
        leftSlideMotor.setTargetPosition(leftSlideAfterSpecimen);
        rightSlideMotor.setTargetPosition(rightSlideAfterSpecimen);
        leftSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlideMotor.setPower(0.9);
        leftSlideMotor.setPower(0.9);
    }
    public int getSlidePositionRight(){return rightSlideMotor.getCurrentPosition();}
    public int getSlidePositionLeft(){return leftSlideMotor.getCurrentPosition();}
    public int getArmPosition(){return armMotor.getCurrentPosition();}

    public double getRightClawPosition()
    {
       return claw.getRightClawPosition();
    }

    public double getLeftClawPosition()
    {
       return claw.getLeftClawPosition();
    }

    public void teleSlideHighBasket()
    {
        leftSlideMotor.setTargetPosition(leftSlideHighBasket);
        rightSlideMotor.setTargetPosition(rightSlideHighBasket);
        leftSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlideMotor.setPower(0.9);
        rightSlideMotor.setPower(0.9);
    }

    public void autoSlideHighSpecimen()
    {
        leftSlideMotor.setTargetPosition(leftSlideSpecimenAuto);
        rightSlideMotor.setTargetPosition(rightSlideSpecimenAuto);
        leftSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlideMotor.setPower(0.9);
        rightSlideMotor.setPower(0.9);

    }
    public void slideHighBasket()
    {
        leftSlideMotor.setTargetPosition(leftAutoSlideHighBasket);
        rightSlideMotor.setTargetPosition(rightAutoSlideHighBasket);
        leftSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlideMotor.setPower(0.9);
        rightSlideMotor.setPower(0.9);
    }

    public void slideLowBasket()
    {

        leftSlideMotor.setTargetPosition(leftSlideLowBasket);
        rightSlideMotor.setTargetPosition(rightSlideLowBasket);
        leftSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlideMotor.setPower(0.9);
        rightSlideMotor.setPower(0.9);
    }

    public void armHighBasketFront()
    {
        armMotor.setTargetPosition(armHighBasketFront);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.2);
    }

    public void armHighBasketFrontTele()
    {
        armMotor.setTargetPosition(armHighBasketTele);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.2);
    }


    public void armLowBasketFront()
    {
        armMotor.setTargetPosition(armLowBasketFront);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.2);
    }


    public void armTelePickup()
    {
        armMotor.setTargetPosition(armPickup);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.3);
    }
    public void armPickup()
    {
        armMotor.setTargetPosition(autoArmPickup);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.3);
    }

    public void pickupOffWall()
    {
        armMotor.setTargetPosition(armSpecimenWall);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.3);

    }

    public void grabSample()
    {
        claw.openClaw();
        armTelePickup();
        while (armMotor.isBusy())
        {
            // while the arm is busy do nothing
        }
//        claw.closeClaw();


    }

    public void armIntoSubmersible()
    {
        armMotor.setTargetPosition(armIntoSubmersible);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.5);
    }

    public void armTeleDriving(){
        armMotor.setTargetPosition(armDriving);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.5);
    }
    public void armDriving(){
        armMotor.setTargetPosition(autoArmDriving);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.5);
    }


    public void armSafety(){
        armMotor.setTargetPosition(armSafetyPosition);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.5);
    }

    public void armLowBar(){
        armMotor.setTargetPosition(armLowBar);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.3);
    }

    public void armSpecimenAuto()
    {
        armMotor.setTargetPosition(armSpecimenAuto);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.3);
    }

    public void highBar(){

        leftSlideMotor.setTargetPosition(slideLeftHighBar);
        rightSlideMotor.setTargetPosition(slideRightHighBar);
        leftSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlideMotor.setPower(0.9);
        rightSlideMotor.setPower(0.9);

        armMotor.setTargetPosition(armHighBar);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.5);
    }

    public void armSpecimenWall(){
        armMotor.setTargetPosition(armSpecimenWall);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.5);
    }

    public void armSpecimenWallAuto(){
        armMotor.setTargetPosition(armSpecimenWallAuto);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.5);
    }

    public void setArmSpecimenGround(){
        armMotor.setTargetPosition(armSpecimenGround);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.5);
    }

    public void armAscent1(){
        armMotor.setTargetPosition(armAscent1);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.15);
    }

//    public boolean slideIsHomed()
//    {
//        return !slideSensor.getState();
//    }

    public boolean armIsHomed()
    {
        return !armSensor.getState();
    }


//    // for forward incremental arm movement
//    public void armRunUp()
//    {
//        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        armMotor.setPower(0.3);
//    }
//    // for backward incremental arm movement
//    public void armRunDown()
//    {
//        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        armMotor.setPower(-0.3);
//    }
//    public void armStop()
//    {
//        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        armMotor.setPower(0);
//    }

    public Action slideAfterSpecimenPickupAction() {
        return new Action() {
            private double beginTs = -1.0;  // timer to track when we started
            private double t = 0.0;         // time this action has been running
            private double dt = 2;        // total time for this action to run

            @Override
            public boolean run(@NonNull TelemetryPacket p) {
                if(beginTs < 0){        // first time to run
                    slidesAfterSpecimenPickup();
                    beginTs = now();    // record time we start running
                } else {
                    t = now()-beginTs;  // how long have we been running
                }
                p.put("Placing sample", t);

                if(t<dt /*&& leftSlideMotor.isBusy() /*|| rightSlideMotor.isBusy())*/)
                {
                    RobotLog.vv("WGW", "Motor is busy, time is: ", t);
                    return true;
                }
                else{
                    beginTs = -1.0;
                    t       = 0.0;
                    RobotLog.vv("WGW", "Action complete slideToHighBasket");
                    return false;
                }
            }



            @Override
            public void preview(Canvas c) {}  // not used, but template required it to.
        };
    }

    public Action slideToHighBasket() {
        return new Action() {
            private double beginTs = -1.0;  // timer to track when we started
            private double t = 0.0;         // time this action has been running
            private double dt = 2;        // total time for this action to run

            @Override
            public boolean run(@NonNull TelemetryPacket p) {
                if(beginTs < 0){        // first time to run
                    slideHighBasket();             // move backdrop servo to drop yellow
                    beginTs = now();    // record time we start running
                } else {
                    t = now()-beginTs;  // how long have we been running
                }
                p.put("Placing sample", t);

                if(t<dt /*&& leftSlideMotor.isBusy() /*|| rightSlideMotor.isBusy())*/)
                {
                    RobotLog.vv("WGW", "Motor is busy, time is: ", t);
                    return true;
                }
                else{
                    beginTs = -1.0;
                    t       = 0.0;
                    RobotLog.vv("WGW", "Action complete slideToHighBasket");
                    return false;
                }
            }



            @Override
            public void preview(Canvas c) {}  // not used, but template required it to.
        };
    }

    public Action armToHighBasket() {
        return new Action() {
            private double beginTs = -1.0;  // timer to track when we started
            private double t = 0.0;         // time this action has been running
            private double dt = 1.0;        // total time for this action to run

            @Override
            public boolean run(@NonNull TelemetryPacket p) {
                if(beginTs < 0){        // first time to run
                    claw.wristHighBasket();
                    armHighBasketFront();             // move backdrop servo to drop yellow
                    beginTs = now();    // record time we start running
                } else {
                    t = now()-beginTs;  // how long have we been running
                }
                p.put("Placing sample", t);

                if(t<dt /*&& armMotor.isBusy()*/)
                {
                    return true;
                }
                else{
                    beginTs = -1.0;
                    t       = 0.0;
                    return false;
                }
            }

            @Override
            public void preview(Canvas c) {}  // not used, but template required it to.
        };
    }

    public Action armToPickUp() {
        return new Action() {
            private double beginTs = -1.0;  // timer to track when we started
            private double t = 0.0;         // time this action has been running
            private double dt = 1.5;        // total time for this action to run

            @Override
            public boolean run(@NonNull TelemetryPacket p) {
                if(beginTs < 0){        // first time to run
                    claw.openClaw();
                    armPickup();             // move backdrop servo to drop yellow

                    beginTs = now();    // record time we start running
                } else {
                    t = now()-beginTs;  // how long have we been running
                }
                p.put("Placing sample", t);

                if(t<dt /*&& armMotor.isBusy()*/)
                {
                    return true;
                }
                else{
                    beginTs = -1.0;
                    t       = 0.0;
                    return false;
                }
            }
            @Override
            public void preview(Canvas c) {}  // not used, but template required it to.
        };
    }

    public Action specimenToHighBarAction() {
        return new Action() {
            private double beginTs = -1.0;  // timer to track when we started
            private double t = 0.0;         // time this action has been running
            private double dt = .18;        // total time for this action to run

            @Override
            public boolean run(@NonNull TelemetryPacket p) {
                if(beginTs < 0){        // first time to run
                    armSpecimenAuto();
                    beginTs = now();    // record time we start running
                } else {
                    t = now()-beginTs;  // how long have we been running
                }
                p.put("Placing sample", t);

                if(t<dt /*&& armMotor.isBusy()*/)
                {
                    return true;
                }
                else{
                    beginTs = -1.0;
                    t       = 0.0;
                    return false;
                }
            }
            @Override
            public void preview(Canvas c) {}  // not used, but template required it to.
        };
    }

    public Action specimenSlidesToHighBarAction() {
        return new Action() {
            private double beginTs = -1.0;  // timer to track when we started
            private double t = 0.0;         // time this action has been running
            private double dt = .35;        // total time for this action to run

            @Override
            public boolean run(@NonNull TelemetryPacket p) {
                if(beginTs < 0){        // first time to run
                    autoSlideHighSpecimen();// move backdrop servo to drop yellow
                    beginTs = now();    // record time we start running
                } else {
                    t = now()-beginTs;  // how long have we been running
                }
                p.put("Placing sample", t);

                if(t<dt /*&& armMotor.isBusy()*/)
                {
                    return true;
                }
                else{
                    beginTs = -1.0;
                    t       = 0.0;
                    return false;
                }
            }
            @Override
            public void preview(Canvas c) {}  // not used, but template required it to.
        };
    }

    public Action armToAscent1() {
        return new Action() {
            private double beginTs = -1.0;  // timer to track when we started
            private double t = 0.0;         // time this action has been running
            private double dt = 2.0;        // total time for this action to run

            @Override
            public boolean run(@NonNull TelemetryPacket p) {
                if(beginTs < 0){        // first time to run
                    armAscent1();             // move backdrop servo to drop yellow
                    beginTs = now();    // record time we start running
                } else {
                    t = now()-beginTs;  // how long have we been running
                }
                p.put("Placing sample", t);

                if(t<dt && armMotor.isBusy())
                {
                    return true;
                }
                else{
                    beginTs = -1.0;
                    t       = 0.0;
                    return false;
                }
            }

            @Override
            public void preview(Canvas c) {}  // not used, but template required it to.
        };
    }

    public Action armWhenDriving() {
        return new Action() {
            private double beginTs = -1.0;  // timer to track when we started
            private double t = 0.0;         // time this action has been running
            private double dt = 1.0;        // total time for this action to run

            @Override
            public boolean run(@NonNull TelemetryPacket p) {
                if(beginTs < 0){        // first time to run
                    armDriving();
                    beginTs = now();    // record time we start running
                } else {
                    t = now()-beginTs;  // how long have we been running
                }
                p.put("Placing sample", t);

                if(t<dt && armMotor.isBusy())
                {
                    return true;
                }
                else{
                    beginTs = -1.0;
                    t       = 0.0;
                    return false;
                }
            }



            @Override
            public void preview(Canvas c) {}  // not used, but template required it to.
        };
    }

    public Action armToSafety() {
        return new Action() {
            private double beginTs = -1.0;  // timer to track when we started
            private double t = 0.0;         // time this action has been running
            private double dt = 1.0;        // total time for this action to run

            @Override
            public boolean run(@NonNull TelemetryPacket p) {
                if(beginTs < 0){        // first time to run
                    claw.closeClaw();
                    claw.setWristPickup();
                    armSafety();             // move backdrop servo to drop yellow
                    beginTs = now();    // record time we start running
                } else {
                    t = now()-beginTs;  // how long have we been running
                }
                p.put("Placing sample", t);

                if(t<dt && armMotor.isBusy())
                {
                    return true;
                }
                else{
                    beginTs = -1.0;
                    t       = 0.0;
                    return false;
                }
            }



            @Override
            public void preview(Canvas c) {}  // not used, but template required it to.
        };
    }

    public Action armToSpecimen() {
        return new Action() {
            private double beginTs = -1.0;  // timer to track when we started
            private double t = 0.0;         // time this action has been running
            private double dt = 1.0;        // total time for this action to run

            @Override
            public boolean run(@NonNull TelemetryPacket p) {
                if(beginTs < 0){        // first time to run
                    highBar();             // move backdrop servo to drop yellow
                    beginTs = now();    // record time we start running
                } else {
                    t = now()-beginTs;  // how long have we been running
                }
                p.put("Placing sample", t);

                if(t<dt /*&& armMotor.isBusy()*/)
                {
                    return true;
                }
                else{
                    beginTs = -1.0;
                    t       = 0.0;
                    return false;
                }
            }

            @Override
            public void preview(Canvas c) {}  // not used, but template required it to.
        };
    }
    public Action deliverSpecimen() {
        return new Action() {
            private double beginTs = -1.0;  // timer to track when we started
            private double t = 0.0;         // time this action has been running
            private double dt = 1.0;        // total time for this action to run

            @Override
            public boolean run(@NonNull TelemetryPacket p) {
                if(beginTs < 0){        // first time to run
                    slidePickup();             // move backdrop servo to drop yellow
                    beginTs = now();    // record time we start running
                } else {
                    t = now()-beginTs;  // how long have we been running
                }
                p.put("Placing Purple", t);

                return t < dt;          // actions are run until they return true;
            }

            @Override
            public void preview(Canvas c) {}  // not used, but template required it to.
        };
    }

    public Action specimenPickup() {
        return new Action() {
            private double beginTs = -1.0;  // timer to track when we started
            private double t = 0.0;         // time this action has been running
            private double dt = 1.0;        // total time for this action to run

            @Override
            public boolean run(@NonNull TelemetryPacket p) {
                if(beginTs < 0){        // first time to run
                    armSpecimenWallAuto();
                    slidePickup();
                    claw.wristSpecimenWallAuto();
                    beginTs = now();    // record time we start running
                } else {
                    t = now()-beginTs;  // how long have we been running
                }
                p.put("Placing sample", t);

                if(t<dt /*&& armMotor.isBusy()*/)
                {
                    return true;
                }
                else{
                    beginTs = -1.0;
                    t       = 0.0;
                    return false;
                }
            }

            @Override
            public void preview(Canvas c) {}  // not used, but template required it to.
        };
    }

    public Action slideToPickUp() {
        return new Action() {
            private double beginTs = -1.0;  // timer to track when we started
            private double t = 0.0;         // time this action has been running
            private double dt = 2.25;        // total time for this action to run

            @Override
            public boolean run(@NonNull TelemetryPacket p) {
                if(beginTs < 0){        // first time to run
                    slidePickup();             // move backdrop servo to drop yellow
                    beginTs = now();    // record time we start running
                } else {
                    t = now()-beginTs;  // how long have we been running
                }
                p.put("Placing sample", t);

                if(t<dt && leftSlideMotor.isBusy() /*|| rightSlideMotor.isBusy())*/)
                {
                    return true;
                }
                else{
                    beginTs = -1.0;
                    t       = 0.0;
                    return false;
                }
            }

            @Override
            public void preview(Canvas c) {}  // not used, but template required it to.
        };
    }


}
