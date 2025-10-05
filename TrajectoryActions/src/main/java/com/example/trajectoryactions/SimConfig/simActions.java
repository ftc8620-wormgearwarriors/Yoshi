package com.example.trajectoryactions.SimConfig;

import static com.acmerobotics.roadrunner.Actions.now;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import java.util.function.Supplier;

public class simActions {
    public Supplier<Action> deliverSample =            () -> new SimTimedAction("Deliver Sample", 3.0);
    public Supplier<Action> pickUpSample  =            () -> new SimTimedAction("Pick Up Sample", 3.0);
    public Supplier<Action> armToPickUp =            () -> new SimTimedAction("Deliver Sample", 3.0);
    public Supplier<Action> armToHighBasket  =            () -> new SimTimedAction("Pick Up Sample", 3.0);
    public Supplier<Action> deliverSpecimen =            () -> new SimTimedAction("Deliver Sample", 3.0);
    public Supplier<Action> armToSpecimen =            () -> new SimTimedAction("Deliver Sample", 3.0);
    public Supplier<Action> armToSafety =            () -> new SimTimedAction("Arm to safety", 1.0);
    public Supplier<Action> armWhenDriving =            () -> new SimTimedAction("Arm for driving", 1.0);
    public Supplier<Action> slideToHighBasket =            () -> new SimTimedAction("Deliver Sample", 3.0);
    public Supplier<Action> slideToPickUp =            () -> new SimTimedAction("Deliver Sample", 3.0);
    public Supplier<Action> armToAscent1 =            () -> new SimTimedAction("Deliver Sample", 3.0);
    public Supplier<Action> wristToAscent1 =            () -> new SimTimedAction("Deliver Sample", 3.0);
    public Supplier<Action> wristToHighBar =            () -> new SimTimedAction("Wrist to high bar", 1.0);
    public Supplier<Action> armToHighBar =            () -> new SimTimedAction("Slides and arm to High Bar", 3.0);
    public Supplier<Action> slidesToHighBar =            () -> new SimTimedAction("Slides and arm to High Bar", 3.0);
    public Supplier<Action> specimenPickUp =            () -> new SimTimedAction("Slides and arm to High Bar", 3.0);
    public Supplier<Action> specimenSlidesPickupSpecimen =            () -> new SimTimedAction("Slides and arm to High Bar", 3.0);



    // sample action
    // this one is used by our simulator instead of moving motors/servos etc.
    public static class  SimTimedAction implements Action  {
        private double beginTs = -1.0;  // timer to track when we started
        private double t = 0.0;         // time this action has been running
        private double dt;        // total time for this action to run
        private final String msg;

        public SimTimedAction(String message, double deltaTime) {
            msg = message;
            dt = deltaTime;
        }

        @Override
        public boolean run(TelemetryPacket p) {
            if(beginTs < 0){        // first time to run
                beginTs = now();    // record time we start running
            } else {
                t = now()-beginTs;  // how long have we been running
            }
            String formatedStr = String.format(msg + " %.2f of ", t); //hijacking to send 2 vals
            p.put(formatedStr, dt);
            return t < dt;          // actions are run until they return false;
        }

        @Override
        public void preview(Canvas c) {}  // not used, but template reequired it to.
    }

}
