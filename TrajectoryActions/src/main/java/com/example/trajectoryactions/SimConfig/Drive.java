package com.example.trajectoryactions.SimConfig;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;

public interface Drive {
    public TrajectoryActionBuilder actionBuilder(Pose2d beginPose);
    public void setPose(Pose2d p);
    public Pose2d getPose();
}
