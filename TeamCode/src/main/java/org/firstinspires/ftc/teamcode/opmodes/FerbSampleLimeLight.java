package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.example.trajectoryactions.wgwFourSpecimen;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.hardware.Robot;

@Config
@Autonomous
public final class FerbSampleLimeLight extends LinearOpMode {


    private int blockId = 0;
    private ElapsedTime tagTimer;

    @Override
    public void runOpMode() throws InterruptedException {
        // sends telemetry signal to robot camera

        MultipleTelemetry multiTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        tagTimer = new ElapsedTime();

        // the start position is set and overrided by the action builder - should be A4
        Robot ferb = new Robot(hardwareMap, new Pose2d(0,0,0));
        ferb.autoInit();

        // set target position for python pipeline
        ferb.prepareGoToPositionDrivePID();

        multiTelemetry.addLine("add robot, Doing init");
        multiTelemetry.update();

        multiTelemetry.addLine("Ready to START!");
        multiTelemetry.update();

        waitForStart();
        ferb.limelight.pipelineSwitch(0); // 0 is python, 1 is not

        ferb.sampleAutoTargeting(ferb.dashboardTelemetry);

    }
}