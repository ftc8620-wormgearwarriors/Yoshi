package com.example.trajectoryactions.SimConfig;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.function.Supplier;

public class SimRobot {
    public String name =  "Unnamed";
    public String robotColor = "#4CAF50";  // changing this color does not work, it is also hard coded in mecanum drive.
    public SimMecanumDrive drive = null;
    private Action currentAction = null;
    private boolean actionStatus = false;
    private final ArrayList<TelemetryPacket> telemetryHistory = new ArrayList<>();
    private boolean enabled;

    public LinkedHashMap<String, Supplier<Action>> paths = new LinkedHashMap<>();

    SimRobot(boolean enabled){
        this.enabled = enabled;
    }

    public String getName(){return name;}

    public boolean getEnabled(){return enabled;}

    public void setAction(String actionName){
        Supplier<Action> path = paths.get(actionName);
        if (path != null) {
            currentAction = paths.get(actionName).get();
            actionStatus = true;
        } else {
            currentAction = null;
            actionStatus = false;
        }
    }

    public boolean runAction(TelemetryPacket p) {
        if (currentAction != null) {
            if (actionStatus) {
                actionStatus = currentAction.run(p);
                if (p.fieldOverlay().getOperations().size() == 0) {
                    p.fieldOverlay().setStroke(robotColor);
                    Drawing.drawRobot(p.fieldOverlay(), drive.getPose());
                }
                return actionStatus;
            } else {
                p.fieldOverlay().setStroke(robotColor);
                Drawing.drawRobot(p.fieldOverlay(), drive.getPose());
                return false;
            }
        } else {
            return false;
        }
    }

    public void preview(TelemetryPacket p) {
        if (currentAction != null) {
            currentAction.preview(p.fieldOverlay());
        }
    }

    public boolean getActionStatus() {
        return actionStatus;
    }


    public void clearTelemetryHistory(){
        telemetryHistory.clear();
    }

    public void putTelemetryHistory(TelemetryPacket p){
        telemetryHistory.add(p);
    }

    public TelemetryPacket getTelemetryHistory(int i){
        if (i<=telemetryHistory.size())
            return telemetryHistory.get(i);
        else
            return new TelemetryPacket(); // return an empty packet.
    }
}
