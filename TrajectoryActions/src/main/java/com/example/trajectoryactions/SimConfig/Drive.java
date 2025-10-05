package com.example.trajectoryactions.SimConfig;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;

public interface Drive {
    public TrajectoryActionBuilder actionBuilder(Pose2d beginPose);
    public Pose2d findEndPos(Action a);
    public void setPose(Pose2d p);
    public Pose2d getPose();
    public void drawRobotWgW(Canvas c, Pose2d t);

}