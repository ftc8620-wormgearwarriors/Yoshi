package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.example.trajectoryactions.SimConfig.simActions;
import com.example.trajectoryactions.wgwABCommon;
import com.example.trajectoryactions.wgwFollowTheLine;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
@Autonomous
public final class FerbFollowTheLine extends LinearOpMode {


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


        multiTelemetry.addLine("add robot, Doing init");
        multiTelemetry.update();

        wgwFollowTheLine ab = new wgwFollowTheLine(ferb.driveTrain);



        multiTelemetry.addLine("Ready to START!");
        multiTelemetry.update();

        waitForStart();

        Actions.runBlocking(ab.followTheLine(ferb.driveTrain));

    }
}