package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.example.trajectoryactions.SimConfig.simActions;
import com.example.trajectoryactions.wgwABCommon;
import com.example.trajectoryactions.wgwBasketPreload;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.vision.VisionPortal;

@Disabled
@Config
@Autonomous
public final class FerbBasketPreload extends LinearOpMode {


    private int blockId = 0;
    private ElapsedTime tagTimer;

    @Override
    public void runOpMode() throws InterruptedException {
        // sends telemetry signal to robot camera

        MultipleTelemetry multiTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        tagTimer = new ElapsedTime();

        // the start position is set and overrided by the action builder
        //TODO accommodate a blue and red side; figure out why set pose isn't working
        Robot ferb = new Robot(hardwareMap, new Pose2d(0,0,Math.toRadians(0)));
        ferb.autoInit();


        multiTelemetry.addLine("add robot, Doing init");
        multiTelemetry.update();

        wgwBasketPreload ab = new wgwBasketPreload(ferb.driveTrain);



        multiTelemetry.addLine("Ready to START!");
        multiTelemetry.update();

        waitForStart();

        Actions.runBlocking(ab.basketPreload(ferb.driveTrain,
                ferb.arm.claw.deliverSample(),ferb.arm.armToHighBasket(),
                ferb.arm.slideToHighBasket(), ferb.arm.armToSafety(),ferb.arm.slideToPickUp(),
                ferb.arm.claw.wristToAscent1(), ferb.arm.armToAscent1())
        );

    }
}