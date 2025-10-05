package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;


import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.hardware.Robot;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="YellowSamplePIDDetector")
public class YellowSamplePIDDetector extends OpMode {

    Robot ferb;
    Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));

    ElapsedTime loopTimer = new ElapsedTime();

    boolean debugMode = false;

    LLResult result;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */

        //letting cam init fully and telling driver not to start
        telemetry.addData(">","DO NOT START YET");
        telemetry.update();

        ferb = new Robot(hardwareMap, startPose);
        ferb.limelight.pipelineSwitch(1); // 0 is python, 1 is not

        //it is done initializing ready to start!
        telemetry.addData(">","READY TO START!");
        telemetry.update();
        ferb.limelight = hardwareMap.get(Limelight3A.class, "limelight");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

    }


    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

        ferb.teleInit();
        ferb.limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        ferb.limelight.pipelineSwitch(0); // 0 is python, 1 is not
        ferb.limelight.start(); // This tells Limelight to start looking!

        // set target position for python pipeline
        ferb.prepareGoToPositionDrivePID();

    }


    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */

    //initialize to fast mode
    double maxVel = 0.7;
    boolean isInited = false;

    // trying a turn multiplier to have better control over turn speed in diff modes
    // double turnControl = 0.8;

    @Override
    public void loop() {
        double loop = loopTimer.milliseconds();
        loopTimer.reset();

        if(!isInited)
        {
            ferb.teleInit();
            isInited = true;
        }


        // get the pid power from the controller used for sample detection
        ferb.updateGoToPositionDrivePID(true, telemetry);

        // new use it to update the average filter and then get the averaged value

        // ******** BEGIN - base drive wheel control  *****
        // Run robot in POV mode - pushing joystick away makes robot move away regardless of rotation
        // note: The joystick goes negative when pushed forwards, so negate it
        //  Find robot's field axes in relation to joystick axes
        //double thetaRadians    = 0;  // temp use 0 for heading
        double thetaRadians     = ferb.getHeading();  // get direction is robot facing
        double joyStick_x_axis = ferb.expDrive(gamepad1.left_stick_x, maxVel);
//        double joyStick_x_axis = -ferb.pidPowerSampleX;
//        double joyStick_y_axis = ferb.expDrive(gamepad1.left_stick_y, maxVel);
        double joyStick_y_axis = -Robot.pidPowerSampleY;

//        double  joystick_turn  = ferb.expDrive(gamepad1.right_stick_x, maxVel*2/3);
        double joystick_turn  = -Robot.pidPowerSampleX;
        double robotStrafe     =  joyStick_x_axis * Math.cos(thetaRadians) + -joyStick_y_axis * Math.sin(thetaRadians);
        double robotForward    =  joyStick_x_axis * Math.sin(thetaRadians) + joyStick_y_axis * Math.cos(thetaRadians);

        // calculate motor powers based on:
        //                  forward/back         turn           strafe
        double frontRight   = robotForward - joystick_turn + robotStrafe;
        double backRight    = robotForward - joystick_turn - robotStrafe;
        double frontLeft    = robotForward + joystick_turn - robotStrafe;
        double backLeft     = robotForward + joystick_turn + robotStrafe;


        // Normal tele controls

        //gamepad1
        double max = Math.max(Math.max(Math.abs(frontLeft), Math.abs(backLeft)), Math.max(Math.abs(frontRight), Math.abs(backRight)));
        if (max > 1) {
            frontLeft /= max;
            backLeft /= max;
            frontRight /= max;
            backRight /= max;
        }

        // directly set motor powers rather than use setDrivePowers() from RR
//        ferb.driveTrain.leftFront.setPower(frontLeft);
//        ferb.driveTrain.leftBack.setPower(backLeft);
//        ferb.driveTrain.rightFront.setPower(frontRight);
//        ferb.driveTrain.rightBack.setPower(backRight);

        // to reset robot heading facing away from driver
        if (gamepad1.y) {
            ferb.resetHeading(Math.toRadians(0));
        }

        // allow user to switch the pipeline
        if (gamepad1.x) {
            int nNewPipeline = (ferb.nWhichLimelightPipeline += 1) % ferb.nNumberLimelightPipelines;
            ferb.setLimeLightPipeLine(nNewPipeline);
        }

        telemetry.addData("PID power x", ferb.pidPowerSampleX);
        telemetry.addData("PID Power y", ferb.pidPowerSampleY);

        //telemetry.addData("heading", ferb.getHeading());

        //telemetry.addData("loopTime", loop);
       // telemetry.addData("ourTime", loopTimer.milliseconds());

        telemetry.update();
    }
}
