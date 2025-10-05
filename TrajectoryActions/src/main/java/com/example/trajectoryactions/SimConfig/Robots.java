package com.example.trajectoryactions.SimConfig;

import com.acmerobotics.roadrunner.Pose2d;
import com.example.trajectoryactions.Paths.ArtifactClosePath;
import com.example.trajectoryactions.Paths.ArtifactFarPath;
import com.example.trajectoryactions.SampleTrajectories.AutoSpecimens;
import com.example.trajectoryactions.SampleTrajectories.YellowSamples;
import com.example.trajectoryactions.SampleTrajectories.commonTrajectories;

import java.util.ArrayList;

public class Robots {

    public ArrayList<SimRobot> simRobots = new ArrayList<>();

    public void setSpeed(int value) {
        commonTrajectories.speed=value;
        SimMecanumDrive.speed=value;
    }

    public enum FieldBackground {
        centerStage,
        intoTheDeep,
        Decode
    }

    public FieldBackground background = FieldBackground.Decode;  // define this variable to set the background

    public Robots() {
        // add puts robot into menu
        // boolean past to constructor
        //   true=enable the robot to run by default
        //   flase= don't run at start, user must enable in menu
        simRobots.add(new RedSideAllYellows(false));
        simRobots.add(new BlueSideAllYellows(false));

        simRobots.add(new RedAutoSpecimens(false));
        simRobots.add(new BlueAutoSpecimens(false));

        simRobots.add(new RedSideArtifactClosePath(true));
        simRobots.add(new RedSideArtifactFarPath(true));

    }

    public void startGameTimer() {

    }

    private class RedSideAllYellows extends SimRobot {

        RedSideAllYellows(boolean enabled) {
            super(enabled);
            drive = new SimMecanumDrive(new Pose2d(0, 0, 0));
            name = "Red Side All Yellows";
            YellowSamples autoYellow = new YellowSamples(drive);
            autoYellow.actionParameters.fieldSide = commonTrajectories.FieldSide.RED;
            paths.put("Sample Yellow", autoYellow::allYellows);
            paths.put("Yellows Long", autoYellow::allYellowsLong);
        }
    }

    private class BlueSideAllYellows extends SimRobot {

        BlueSideAllYellows(boolean enabled) {
            super(enabled);
            drive = new SimMecanumDrive(new Pose2d(0, 0, 0));
            name = "Blue Side All Yellows";
            YellowSamples autoYellow = new YellowSamples(drive);
            autoYellow.actionParameters.fieldSide = commonTrajectories.FieldSide.BLUE;
            paths.put("Sample Yellow", autoYellow::allYellows);
            paths.put("Yellows Long", autoYellow::allYellowsLong);
        }
    }

    private class RedAutoSpecimens extends SimRobot {

        RedAutoSpecimens(boolean enabled) {
            super(enabled);
            drive = new SimMecanumDrive(new Pose2d(0, 0, 0));
            name = "Red Specimens";
            AutoSpecimens autoSpecimens = new AutoSpecimens(drive);
            autoSpecimens.actionParameters.fieldSide = commonTrajectories.FieldSide.RED;
            paths.put("Sample Yellow", autoSpecimens::allSpecimens);
        }
    }

    private class BlueAutoSpecimens extends SimRobot {

        BlueAutoSpecimens(boolean enabled) {
            super(enabled);
            drive = new SimMecanumDrive(new Pose2d(0, 0, 0));
            name = "Blue Specimens";
            AutoSpecimens autoSpecimens = new AutoSpecimens(drive);
            autoSpecimens.actionParameters.fieldSide = commonTrajectories.FieldSide.BLUE;
            paths.put("Sample Yellow", autoSpecimens::allSpecimens);
        }
    }

    private class RedSideArtifactClosePath extends SimRobot {

        RedSideArtifactClosePath(boolean enabled) {
            super(enabled);
            drive = new SimMecanumDrive(new Pose2d(0, 0, 0));
            name = "Red Side Artifact Close Path";
            ArtifactClosePath artifactRedClose  = new ArtifactClosePath(drive);
            artifactRedClose.actionParameters.fieldSide = commonTrajectories.FieldSide.RED;
            paths.put("artifact Red Close", artifactRedClose::allArtifacts);
        }
    }
    private class RedSideArtifactFarPath extends SimRobot {

        RedSideArtifactFarPath(boolean enabled) {
            super(enabled);
            drive = new SimMecanumDrive(new Pose2d(0, 0, 0));
            name = "Red Side Artifact Far Path";
            ArtifactFarPath artifactRedFar = new ArtifactFarPath(drive);
            artifactRedFar.actionParameters.fieldSide = commonTrajectories.FieldSide.RED;
            paths.put("artifact Red Far", artifactRedFar::allArtifacts);
        }
    }
}