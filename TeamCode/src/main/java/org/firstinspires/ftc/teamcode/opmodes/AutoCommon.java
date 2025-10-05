package org.firstinspires.ftc.teamcode.opmodes;//package org.firstinspires.ftc.teamcode.opmodes;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.example.trajectoryactions.wgwABCommon;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.hardware.Robot;
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.opencv.core.Rect;
//
//public class AutoCommon extends LinearOpMode {
//
//    protected VisionPortal visionPortalSpike;
//    protected VisionPortal visionPortalRobot;
//
//
//    public enum WhichRobotDetector {BACKDROP, TRUSSES_NEAR, TRUSSES_FAR, UNKNOWN}
//
//
//    protected int blockId = 0;
//    protected ElapsedTime tagTimer;
//
//    wgwABCommon ab;
//
//    Robot ferb;
//
//    public void commonAutoInit(Rect propRectLeft, Rect propRectRight, Rect propRectMiddle,
//                               wgwABCommon.FieldSide fieldSide) {
//
//        MultipleTelemetry multiTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//
//        tagTimer = new ElapsedTime();
//
//        // the start position is set and overrided by the action builder - should be A4
//        ferb = new Robot(hardwareMap, new Pose2d(0, 0, 0));
//
//        multiTelemetry.addLine("add robot, Doing init");
//        multiTelemetry.addLine("Right Trigger Sets drone");
//        multiTelemetry.addLine("x exits out");
//        multiTelemetry.update();
//
//
//
//        ab = new wgwABCommon(ferb.driveTrain);
//
//        while (!gamepad1.x && !opModeIsActive()) {
//            fer.autoManualInit(gamepad1, telemetry);
//        }
//
//        while ((!isStopRequested()) && (!opModeIsActive()))
//        {
//            telemetry.addData("Parking Spot Selected", miniMe.parkingPosition);
//            telemetry.addData("drone loaded", miniMe.droneLoaded);
//            spikeNumber = spikeBlobProcessor.getSelection();
//            multiTelemetry.addData("selectedSpike during init",spikeBlobProcessor.getSelection());
//            miniMe.spikeMarkLED(spikeNumber);
//            multiTelemetry.update();
//        }
//
//        miniMe.spikeMarkLED(spikeNumber);
//        multiTelemetry.addLine("Ready to START!");
//        multiTelemetry.update();
//
//        waitForStart();
//        spikeNumber = spikeBlobProcessor.getSelection();
//        multiTelemetry.addData("selectedSpike Starting",spikeBlobProcessor.getSelection());
//        multiTelemetry.update();
//        //This is so robot detection doesn't shut off spike processor so randomization occurs
//        visionPortalSpike.close();
//        visionPortalRobot = VisionPortal.easyCreateWithDefaults(miniMe.webcamName, robotDetectionProcessor);
//
//        blockId = miniMe.findBlockID(fieldSide, spikeNumber);
//    }
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//    }
//}