package com.example.trajectoryactions.SimConfig;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

// this action can be inserted into any trajectory sequence to send 'special' text
// to the simulator so that it starts the playback at that location.
// This saves development time by letting you see the area of concern immediately instead
// of waiting up to 30 seconds of simulation to see that point in your trajectory sequence.
public class SimViewStartMarker implements Action {
    public static String SimStartSimHereKey = "SimstartSimHere";

    @Override
    public boolean run(TelemetryPacket p) {
        p.put(SimStartSimHereKey, 0);
        return false;   // only needs to run one time to send the marker.
    }

    @Override
    public void preview(Canvas c) {}  // not used, but template required it to.

}
