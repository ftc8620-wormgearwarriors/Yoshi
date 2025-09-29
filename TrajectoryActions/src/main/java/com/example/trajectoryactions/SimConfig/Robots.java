package com.example.trajectoryactions.SimConfig;

import com.acmerobotics.roadrunner.Pose2d;
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
        intoTheDeep
    }

    public FieldBackground background = FieldBackground.intoTheDeep;  // define this variable to set the background

    public Robots() {
        // add puts robot into menu
        // boolean past to constructor
        //   true=enable the robot to run by default
        //   flase= don't run at start, user must enable in menu
        simRobots.add(new RedSideAllYellows(true));
        simRobots.add(new BlueSideAllYellows(true));

        simRobots.add(new RedAutoSpecimens(true));
        simRobots.add(new BlueAutoSpecimens(true));
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
}