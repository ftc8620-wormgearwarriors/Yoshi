package com.example.trajectoryactions;

import static com.acmerobotics.roadrunner.Actions.now;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.example.trajectoryactions.SimConfig.Drive;

import org.jetbrains.annotations.NotNull;

public class wgwABCommon {
    static public Pose2d startPosA5= new Pose2d( 39.5,  62.5, Math.toRadians(180));
    static public Pose2d startPosA2= new Pose2d( 0,  60, Math.toRadians(270));
    static public Pose2d startCenterField = new Pose2d(0, 0, Math.toRadians(180));

    protected double basketX = 55;
    protected double basketY = 55;

    public enum FieldSide {RED, BLUE}
    public enum ParkingPosState {CORNER, MIDDLE, UNKNOWN}


    private Drive drive = null;  // this was static but that won't work with multiple robots in simulator

    public wgwABCommon(Drive d){
        drive = d;
    }

    // sample action
    // this one is used by our simulator instead of moving motors/servos etc.
    class  SimTimedAction implements Action  {
            private double beginTs = -1.0;  // timer to track when we started
            private double t = 0.0;         // time this action has been running
            private double dt;        // total time for this action to run
            private final String msg;

            SimTimedAction(String message, double deltaTime) {
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


    class driveToX implements Action{
        private boolean firstRun = true;
        private Drive drive;
        private double distance;
        private Action action;
        public driveToX(Drive drive, double distance){
            this.drive = drive;
            this.distance = distance;
        }
        @Override
        public boolean run(@NotNull TelemetryPacket telemetryPacket) {
            if (firstRun){
                firstRun = false;
                Pose2d pose = drive.getPose();
                TrajectoryActionBuilder trajActBuilder = drive.actionBuilder(pose);
                action = trajActBuilder
                        .lineToY((pose.position.x + distance))
                        .build();
            }
            return action.run(telemetryPacket);
        }
    }

//    // needs to be in this or other common class accessible by robot & simulator
//    // initialize the actions to simulator actions. robot.init can change these to robot actions
//    public class actionParameters {
//        public Action deliverSample = new SimTimedAction("Deliver Sample", 3.0);
//    }

    double beginGameTimes = -1.0;  // timer to track when we started
    public void startGameTimer(){
        beginGameTimes = now();
    }

    // first call to the run method starts the timer, but returns false so action will not be called again
    // future calls to run will return true until set game time has been reached.
    class  GameTimerAction implements Action  {
        private double t = 0.0;         // time this action has been running
        private double dt;        // total time for this action to run

        GameTimerAction(double deltaTime) {
            dt = deltaTime;
        }

        @Override
        public boolean run(TelemetryPacket p) {
            if(beginGameTimes < 0) {        // Game timers should have already been started
                return false;               // bail out they did not start game timer
            }
            t = now()-beginGameTimes;  // how long have we been running
            String formatedStr = String.format("Sleep until game time %.2f of ", t); //hijacking to send 2 vals
            p.put(formatedStr, dt);
            return t < dt;          // actions are run until they return false;
        }

        @Override
        public void preview(Canvas c) {}  // not used, but template reequired it to.
    }

    /***** student trajectories below here *********************************************************/
//    // empty method to override for nearside class
//    public SequentialAction close(Action dropPurple, Action dropYellow,
//                                  Action liftStateMachineDeliverAction, Action liftStateMachineCollectAction,
//                                  FieldSide fieldSide, SelectedSpike selectedSpike, Action tagDetection)
//    {
//        SequentialAction retVal;
//        retVal = new SequentialAction();
//        return retVal;
//    }
//
//    // empty method to override for farside class
//    public SequentialAction far(Action dropPurple, Action dropYellow,
//                                Action liftStateMachineDeliverAction, Action liftStateMachineCollectAction,
//                                FieldSide fieldSide, SelectedSpike selectedSpike, Action tagDetection,
//                                Action waitForRobotAction, Action clawSwitchAction)
//    {
//        SequentialAction retVal;
//        retVal = new SequentialAction();
//        return retVal;
//    }



    Action startToBasket (Drive drive, Pose2d startPos, FieldSide fieldSide)
    {
        TrajectoryActionBuilder trajActBuilder = drive.actionBuilder(startPos);
        int dir = (fieldSide == FieldSide.BLUE) ? 1 : -1;
        return ( trajActBuilder
                .setReversed(false)
                .lineToY(12 * dir)
                .build());
    }


    ParallelAction timers()
    {
        ParallelAction retVal;
        retVal = new ParallelAction(
                new SimTimedAction("this is timer 1", 7),
                new SimTimedAction("this is timer 2", 13)
        );
        return retVal;
    }

    public Action toAscent(Drive drive, Pose2d startPos, wgwABCommon.FieldSide fieldSide) {
        TrajectoryActionBuilder trajActBuilder = drive.actionBuilder(startPos);
        int dir = (fieldSide == FieldSide.BLUE) ? 1 : -1;
        TranslationalVelConstraint translationalVelConstraint = new TranslationalVelConstraint(34.0);
        return (trajActBuilder
                .setTangent(180)
                .splineToSplineHeading(new Pose2d(37 * dir, 27 * dir, Math.toRadians(270 * dir)), Math.toRadians(270 * dir), translationalVelConstraint)
                .splineToLinearHeading(new Pose2d(23 * dir, 4 * dir, Math.toRadians(180 * dir)), Math.toRadians(180 * dir), translationalVelConstraint)
                .build());
    }

    public Action toBasket(Drive drive, Pose2d startPos, FieldSide fieldSide) {
        TrajectoryActionBuilder trajActBuilder = drive.actionBuilder(startPos);
        int dir = (fieldSide == FieldSide.BLUE) ? 1 : -1;
        return (trajActBuilder
                .setTangent(Math.toRadians(315 * dir))
                .splineToSplineHeading(new Pose2d(basketX * dir,basketY * dir, Math.toRadians(225 * dir)), Math.toRadians(0 * dir))
                .build());
    }

    public Action toPark(Drive drive, Pose2d startPos, FieldSide fieldSide) {
        TrajectoryActionBuilder trajActBuilder = drive.actionBuilder(startPos);
        int dir = (fieldSide == FieldSide.BLUE) ? 1 : -1;
        return (trajActBuilder
                .setTangent(Math.toRadians(90 * dir))
                .splineToLinearHeading(new Pose2d(-45  * dir, 55 * dir, Math.toRadians(270 * dir)), Math.toRadians(180 * dir))
                .build());
    }

    public Action toRigging(Drive drive, Pose2d startPos, FieldSide fieldSide, double specimenXValue) {

        TrajectoryActionBuilder trajActBuilder = drive.actionBuilder(startPos);
        int dir = (fieldSide == FieldSide.BLUE) ? 1 : -1;
        return (trajActBuilder
                .splineToLinearHeading(new Pose2d( specimenXValue  * dir, 29 * dir, Math.toRadians(270 * dir)), Math.toRadians(270 * dir))
                .build());
    }

    public Action toRiggingFromPickup(Drive drive, Pose2d startPos, FieldSide fieldSide) {
        TrajectoryActionBuilder trajActBuilder = drive.actionBuilder(startPos);
        int dir = (fieldSide == FieldSide.BLUE) ? 1 : -1;
        return (trajActBuilder
                .splineToLinearHeading(new Pose2d(0  * dir, 33 * dir, Math.toRadians(90 * dir)), Math.toRadians(270 * dir))
                .build());
    }

    public Action backUpFromRigging(Drive drive, Pose2d startPos, FieldSide fieldSide, double specimenXValue) {
        TrajectoryActionBuilder trajActBuilder = drive.actionBuilder(startPos);
        int dir = (fieldSide == FieldSide.BLUE) ? 1 : -1;
        return (trajActBuilder
                //.lineToY(35)
                .setTangent(Math.toRadians(90 * dir))
                .splineToSplineHeading(new Pose2d(specimenXValue * dir, 37 * dir,Math.toRadians(270 * dir)), Math.toRadians(90 * dir))

                .build());
    }


    public Action toSpecimenPickup(Drive drive, Pose2d startPos, FieldSide fieldSide) {
        TrajectoryActionBuilder trajActBuilder = drive.actionBuilder(startPos);
        int dir = (fieldSide == FieldSide.BLUE) ? 1 : -1;
        return (trajActBuilder
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(-45  * dir, 55 * dir, Math.toRadians(270 * dir)), Math.toRadians(90 * dir))
                .build());
    }

    public Action toSpikeMarkFarFromRigging(Drive drive, Pose2d startPos, FieldSide fieldSide) {
        TrajectoryActionBuilder trajActBuilder = drive.actionBuilder(startPos);
        int dir = (fieldSide == FieldSide.BLUE) ? 1 : -1;
        return (trajActBuilder
                .splineToLinearHeading(new Pose2d(-33  * dir, 40 * dir, Math.toRadians(90 * dir)), Math.toRadians(180 * dir))
                .splineToLinearHeading(new Pose2d(-48  * dir, 45 * dir, Math.toRadians(90 * dir)), Math.toRadians(270 * dir))

                .build());
    }
}


