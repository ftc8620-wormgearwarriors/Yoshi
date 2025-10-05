package org.firstinspires.ftc.teamcode.hardware;

import static com.acmerobotics.roadrunner.Actions.now;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {


    Robot ferb = null;
    // declares servos for claw
    Servo rightClaw = null;
    Servo leftClaw = null;
    public Servo wristServo = null;

    // declares and assigns positions for servos
    // prior to wrist change wrist start was .59
    private double wristStart           = 0.521; // wrist start + 0 (this one is "safe" at beginning of teleop)
    private double wristInit            = 0.04; // wrist start - 0.481


    private double openRightClaw       = 0.50;
    public double closeRightClaw      = 0.22;
    private double openLeftClaw        = 0.48;
    public double closeLeftClaw       = 0.73;

    // used in auto
    public double wristHighBarAuto    = 0.52; // same as wrist start


    private double wristSafeDriving    = 0.581; // wrist start + 0.06
    private double wristHighBasket     = 0.34; // wrist start - 0.181
    private double wristLowBasket      = 0.34; // wrist start - 0.181

    private double wristLowBar         = 0.801; // wrist start + 0.28
    private double wristHighBar        = 0.3893; // wrist start - 0.1317
    private double wristSpecimenWall   = 0.881; // wrist start + 0.36
    private double wristSpecimenWallAuto   = 0.801; // wrist start + .28

    private double wristSamplePickup   = 0.5187; // wrist start - .0023 //.5599
    private double wristSamplePrePickup  = 0.62; // wrist start + .099
    private double wristSpecimenGround = 0.649; // wrist start + 0.128

    private double wristAscent1        = 0.561; // wrist start 0.04

    private HardwareMap m_hwMap     = null;

   public Claw(HardwareMap hardwareMap, Robot thisRobot)
   {
       // assigns the hardware map and makes claw servos for config
       m_hwMap = hardwareMap;
       rightClaw = m_hwMap.get(Servo.class, "clawRight");
       leftClaw = m_hwMap.get(Servo.class, "clawLeft");
       wristServo = m_hwMap.get(Servo.class, "wristServo");
       ferb = thisRobot;
   }

    public void init()
    {
        //sets the claw/wrist positions to the init values
        rightClaw.setPosition(closeRightClaw);
        leftClaw.setPosition(closeLeftClaw);
        wristServo.setPosition(wristInit);
    }
   public void start()
    {
        //sets the claw/wrist positions to the init values
        rightClaw.setPosition(closeRightClaw);
        leftClaw.setPosition(closeLeftClaw);
        wristServo.setPosition(wristStart);
    }

    public double getRightClawPosition()
    {
        return rightClaw.getPosition();
    }

    public double getLeftClawPosition()
    {
        return leftClaw.getPosition();
    }

    public double getWristPosition()
    {
        return wristServo.getPosition();
    }

    public void closeClaw()
    {
        rightClaw.setPosition(closeRightClaw);
        leftClaw.setPosition(closeLeftClaw);
        ferb.ledStrip.updateLED(ferb.colorDetector.colorIs());
    }

    public void openClaw()
    {
        rightClaw.setPosition(openRightClaw);
        leftClaw.setPosition(openLeftClaw);
        ferb.ledStrip.updateLED(ferb.colorDetector.colorIs());
    }

    public void wristSafeDriving()
    {
        wristServo.setPosition(wristSafeDriving);
    }

    public void setWristHighBarAuto()
    {
        wristServo.setPosition(wristHighBarAuto);
    }

    public void setWristSamplePickup()
    {
        wristServo.setPosition(wristSamplePickup);
    }

    public void setWristSamplePrePickup()
    {
        wristServo.setPosition(wristSamplePrePickup);
    }

    public void setWristAscent1(){wristServo.setPosition(wristAscent1);}

    public void openRightClaw()
    {
        rightClaw.setPosition(openRightClaw);
    }

    public void closeRightClaw()
    {
        rightClaw.setPosition(closeRightClaw);
    }

    public void openLeftClaw()
    {
        leftClaw.setPosition(openLeftClaw);
    }
    public void closeLeftClaw()
    {
        leftClaw.setPosition(closeLeftClaw);
    }

    public void setWristPickup()
    {
       wristServo.setPosition(wristSafeDriving);
    }


    public void wristHighBasket()
    {
        wristServo.setPosition(wristHighBasket);
    }

    public void setWristLowBasket()
    {
        wristServo.setPosition(wristLowBasket);
    }

    public void setWristLowBar()
    {
        wristServo.setPosition(wristLowBar);
    }

    public void setWristHighBar()
    {
        wristServo.setPosition(wristHighBar);
    }

    public void wristSpecimenWall()
    {
        wristServo.setPosition(wristSpecimenWall);
    }

    public void wristSpecimenWallAuto()
    {
        wristServo.setPosition(wristSpecimenWallAuto);
    }

    public void setWristSpecimenGround()
    {
        wristServo.setPosition(wristSpecimenGround);
    }

    public Action pickupSampleAction() {
        return new Action() {
            private double beginTs = -1.0;  // timer to track when we started
            private double t = 0.0;         // time this action has been running
            private double dt = 0.4;        // total time for this action to run

            @Override
            public boolean run(@NonNull TelemetryPacket p) {
                if(beginTs < 0){        // first time to run
                    closeClaw();
                    beginTs = now();    // record time we start running
                } else {
                    t = now()-beginTs;  // how long have we been running
                }
                p.put("Placing Purple", t);

                if(t<dt)
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
            public void preview(Canvas c) {}  // not used, but template reequired it to.
        };
    }

    public Action deliverSample() {
        return new Action() {
            private double beginTs = -1.0;  // timer to track when we started
            private double t = 0.0;         // time this action has been running
            private double dt = 0.3;        // total time for this action to run

            @Override
            public boolean run(@NonNull TelemetryPacket p) {
                if(beginTs < 0){        // first time to run
                    openClaw();             // move backdrop servo to drop yellow
                    beginTs = now();    // record time we start running
                } else {
                    t = now()-beginTs;  // how long have we been running
                }
                p.put("Dropping sample", t);

                if(t<dt)
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

    public Action wristToHighBarAction() {
        return new Action() {
            private double beginTs = -1.0;  // timer to track when we started
            private double t = 0.0;         // time this action has been running
            private double dt = 0.3;        // total time for this action to run

            @Override
            public boolean run(@NonNull TelemetryPacket p) {
                if(beginTs < 0){        // first time to run
                    setWristHighBarAuto();
                    beginTs = now();    // record time we start running
                } else {
                    t = now()-beginTs;  // how long have we been running
                }
                p.put("Dropping sample", t);

                if(t<dt)
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



    public Action wristToAscent1() {
        return new Action() {
            private double beginTs = -1.0;  // timer to track when we started
            private double t = 0.0;         // time this action has been running
            private double dt = 1.0;        // total time for this action to run

            @Override
            public boolean run(@NonNull TelemetryPacket p) {
                if(beginTs < 0){        // first time to run
                    //closeClaw();
                    setWristAscent1();             // move backdrop servo to drop yellow
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


}


