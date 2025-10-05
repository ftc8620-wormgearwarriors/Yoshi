package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.hardware.Robot;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp")
public class TeleOp extends OpMode {

    Robot ferb;

    Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));

    ElapsedTime loopTimer = new ElapsedTime();

    boolean debugMode = false;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();


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

    //it is done initializing ready to start!
    telemetry.addData(">","READY TO START!");
    telemetry.update();
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

        // ******** BEGIN - base drive wheel control  *****
        // Run robot in POV mode - pushing joystick away makes robot move away regardless of rotation
        // note: The joystick goes negative when pushed forwards, so negate it
        //  Find robot's field axes in relation to joystick axes
        //double thetaRadians    = 0;  // temp use 0 for heading
        double thetaRadians     = ferb.getHeading();  // get direction is robot facing
        double joyStick_x_axis = ferb.expDrive(-gamepad1.left_stick_x, maxVel);
        double joyStick_y_axis = ferb.expDrive(-gamepad1.left_stick_y, maxVel);
        double  joystick_turn  = ferb.expDrive(gamepad1.right_stick_x, maxVel*2/3);
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
        ferb.driveTrain.leftFront.setPower(frontLeft);
        ferb.driveTrain.leftBack.setPower(backLeft);
        ferb.driveTrain.rightFront.setPower(frontRight);
        ferb.driveTrain.rightBack.setPower(backRight);

        // regular/slow mode
        if (gamepad1.dpad_down)
        {
            maxVel = 0.4;
        }

        // speed mode
        else if (gamepad1.dpad_right)
        {
            maxVel = 0.7;
        }

        // ultimate speed mode
        else if (gamepad1.dpad_up)
        {
            maxVel = 1.0;
        }

        // to reset robot heading facing toward driver
        if (gamepad1.x) {
            ferb.resetHeading(Math.toRadians(180));
        }

        // to reset robot heading facing away from driver
        if (gamepad1.y) {
            ferb.resetHeading(Math.toRadians(0));
        }

        // open both claw sides together
        if (gamepad1.a && !gamepad1.start)
        {
            ferb.arm.claw.openClaw();
        }

        // close both claw sides together
        if (gamepad1.b && !gamepad1.start)
        {
            ferb.arm.claw.closeClaw();
        }

        // wrist for picking up sample
        if (gamepad1.right_bumper)
        {
            ferb.arm.claw.setWristSamplePickup();
        }

        // move arm, wrist, and slide to safe driving
        if (gamepad1.left_bumper)
        {
            ferb.arm.slidePickup();
            ferb.arm.armTeleDriving(); // arm moves to a when driving pos before it actually picks up lower
            ferb.arm.claw.wristSafeDriving();
            ferb.isDeliveringToBasket = false;
        }
        if (gamepad1.options)
        {
            debugMode = true;
        }

        // turn on debug telemetry
        if (gamepad1.dpad_up && gamepad1.start && !debugMode) {  // only run once
            debugMode = true;

            telemetry.addData(">"," Debug Mode Enabled");
            telemetry.update();
        }

        // speed controls
        if (gamepad1.right_trigger > 0.4){
            maxVel = 0.4;
        } else {
            maxVel = 0.7;
        }

        // gamepad 2 controls

        // high basket front
       if (gamepad2.a && !gamepad2.start)
       {
           ferb.arm.teleSlideHighBasket();
           ferb.arm.armHighBasketFrontTele();
           ferb.arm.claw.wristHighBasket();
           ferb.isDeliveringToBasket = true;
       }

       // low basket front
       if (gamepad2.b && !gamepad2.start)
       {
           ferb.arm.slideLowBasket();
           ferb.arm.armLowBasketFront();
           ferb.isDeliveringToBasket = true;
       }

       // moves amr to pick up
        if (gamepad2.x)
        {
            ferb.arm.armLowBar();
            ferb.arm.claw.setWristLowBar();
        }

        // move assembly into the submersible position
        if (gamepad2.y)
        {
            ferb.arm.highBar();
            ferb.arm.claw.setWristHighBar();
        }

        // incremental wrist movement backward
        if (gamepad2.right_stick_y < -0.5)
        {
            ferb.arm.claw.wristServo.setPosition(ferb.arm.claw.wristServo.getPosition() + 0.02);
        }

        // incremental wrist movement forward
        if (gamepad2.right_stick_y > 0.5)
        {
            ferb.arm.claw.wristServo.setPosition(ferb.arm.claw.wristServo.getPosition() - 0.02);
        }

        // incremental slide motor movement up
        if (gamepad2.dpad_up)
        {
            ferb.arm.leftSlideMotor.setTargetPosition(ferb.arm.leftSlideMotor.getCurrentPosition() + ferb.slideIncrement);
            ferb.arm.leftSlideMotor.setPower(0.7);

            ferb.arm.rightSlideMotor.setTargetPosition(ferb.arm.rightSlideMotor.getCurrentPosition() + ferb.slideIncrement);
            ferb.arm.rightSlideMotor.setPower(0.7);
        }
        // incremental slide motor movement down
        else if (gamepad2.dpad_down)
        {
            ferb.arm.leftSlideMotor.setTargetPosition(ferb.arm.leftSlideMotor.getCurrentPosition() - ferb.slideIncrement);
            ferb.arm.leftSlideMotor.setPower(0.7);

            ferb.arm.rightSlideMotor.setTargetPosition(ferb.arm.rightSlideMotor.getCurrentPosition() - ferb.slideIncrement);
            ferb.arm.rightSlideMotor.setPower(0.7);
        }

        // incremental arm motor movement forward
        if (gamepad2.dpad_left)
        {
            ferb.arm.armMotor.setTargetPosition(ferb.arm.armMotor.getCurrentPosition() + ferb.armIncrement);
            ferb.arm.armMotor.setPower(0.5);
        }

        // incremental arm motor movement backward
        else if (gamepad2.dpad_right)
        {
            ferb.arm.armMotor.setTargetPosition(ferb.arm.armMotor.getCurrentPosition() - ferb.armIncrement);
            ferb.arm.armMotor.setPower(0.5);
        }

        if (gamepad2.right_bumper){
            ferb.arm.claw.start();
            ferb.arm.armTeleDriving();
            ferb.arm.slidePickup();
            ferb.arm.claw.wristSafeDriving();
        }

        if (gamepad2.left_bumper){
            ferb.arm.slideHoming();
            ferb.arm.homing();
            ferb.ledStrip.setGreen();

        }

        if (gamepad2.left_trigger > 0.4){
            ferb.arm.slidePickup();
            ferb.arm.pickupOffWall();
            ferb.arm.claw.wristSpecimenWall();
        }

        if (gamepad2.right_trigger > 0.4){
            ferb.arm.claw.setWristSamplePrePickup();
            ferb.arm.armTelePickup();
        }

        // comment this out for now because it may cause delays in the loop??
        // for led control
        // ferb.ledStrip.updateLED(ferb.colorDetector.colorIs());

        // extra information for debug mode in teleop
        if (debugMode)
        {
            telemetry.addData("Blue",ferb.colorDetector.blue());
            telemetry.addData("Green",ferb.colorDetector.green());
            telemetry.addData("Red",ferb.colorDetector.red());
            telemetry.addData("IS HOMED", ferb.arm.isArmHomedOnInit);
            telemetry.addData("slide position Left", ferb.arm.getSlidePositionLeft());
            telemetry.addData("slide position Right", ferb.arm.getSlidePositionRight());
            telemetry.addData("arm motor position", ferb.arm.getArmPosition());
            telemetry.addData("wrist servo position", ferb.arm.claw.getWristPosition());
            telemetry.addData("clawLeftServo", ferb.getLeftClawPosition());
            telemetry.addData("clawRightServo", ferb.getRightClawPosition());
            telemetry.addData("colorHolding",ferb.colorDetector.colorIs());
            telemetry.addData("Distance",ferb.colorDetector.distance());
            telemetry.addData("isAbove",ferb.colorDetector.isAbove());
            telemetry.addData("Left Current", ferb.arm.leftSlideMotor.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Right Current", ferb.arm.rightSlideMotor.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("loopTime", loop);
            telemetry.addData("ourTime", loopTimer.milliseconds());
        }

        // these messages are displayed all the time, not just during debug mode
        telemetry.addData("IS HOMED", ferb.arm.isArmHomedOnInit);
        telemetry.addData("arm motor position", ferb.arm.getArmPosition());
        telemetry.addData("Left Current", ferb.arm.leftSlideMotor.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Right Current", ferb.arm.rightSlideMotor.getCurrent(CurrentUnit.AMPS));
        RobotLog.vv("WGW motor current", "Left slide motor current: " +
                ferb.arm.leftSlideMotor.getCurrent(CurrentUnit.AMPS));
        RobotLog.vv("WGW motor current", "Right slide motor current: " +
                ferb.arm.rightSlideMotor.getCurrent(CurrentUnit.AMPS));

        telemetry.addData("left front drive Current", ferb.driveTrain.leftFront.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("right front drive Current", ferb.driveTrain.rightFront.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("left back drive Current", ferb.driveTrain.leftBack.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("right back drive Current", ferb.driveTrain.rightBack.getCurrent(CurrentUnit.AMPS));

        dashboardTelemetry.addData("left front drive Current", ferb.driveTrain.leftFront.getCurrent(CurrentUnit.AMPS));
        dashboardTelemetry.addData("right front drive Current", ferb.driveTrain.rightFront.getCurrent(CurrentUnit.AMPS));
        dashboardTelemetry.addData("left back drive Current", ferb.driveTrain.leftBack.getCurrent(CurrentUnit.AMPS));
        dashboardTelemetry.addData("right back drive Current", ferb.driveTrain.rightBack.getCurrent(CurrentUnit.AMPS));

        telemetry.update();
    }
}
