package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import androidx.annotation.NonNull;
@Config
public class Robot {

    // declare hardware classes belonging to robot
//    Drive drive = null;
    public Arm arm = null;
    public MecanumDrive driveTrain = null;
    public ColorDetector colorDetector;
    public LED ledStrip;
    public Limelight3A limelight;
    public AverageFilter averageFilterX;
    public AverageFilter averageFilterY;
    public double yellowSampleDrivePowerX;
    public double yellowSampleDrivePowerY;
    public double yellowSampleAveragedDrivePowerX;
    public double yellowSampleAveragedDrivePowerY;
    public Servo headLight = null;
    public Servo colorLight = null;



    // member variables
    public HardwareMap m_hwMap = null;

    // dashboard telemetry variables
    FtcDashboard dashboard = FtcDashboard.getInstance();
    public Telemetry dashboardTelemetry = dashboard.getTelemetry();

    public enum SampleColor {RED, BLUE, YELLOW, ABOVE, NA}

    // action for finding the sample
    private Action sampleMove;

    public int armIncrement;
    public int slideIncrement;

    public boolean isDeliveringToBasket;

    // PID controller and member variables
    SimplePIDControl pidControlX;
    SimplePIDControl pidControlY;
    public static double p_goToX     = 3;
    public static double i_goToX     = 0;
    public static double d_goToX     = 0;
    public static double p_goToY     = 1;
    public static double i_goToY     = 0;
    public static double d_goToY     = 0;
    public static double maxDiffPower = 0.5;
    double pidPower;
    public int nWhichLimelightPipeline;
    public int nNumberLimelightPipelines;

    public static double feedForward = 0;
    public static double targetPositionX = 640;
    public static double targetPositionY = 760;
    public static double errorThreshold = 10;

    // for non-python pipeline with tx and ty
    //    public static int    maxTicsX = 200;
    //    public static int    maxTicsY = 400;

    // for python pipeline with center of sample
    public static int    maxTicsX = 1000;
    public static int    maxTicsY = 1000;

    public static double pidPowerSampleX;
    public static double pidPowerSampleY;


    public Robot(HardwareMap hardwareMap, Pose2d initialPose){

        // instantiates claw for this class
        m_hwMap = hardwareMap;
        arm = new Arm(hardwareMap, this);
        arm.claw = new Claw(hardwareMap, this);
        ledStrip = new LED(hardwareMap);
        pidControlX = new SimplePIDControl(p_goToX, i_goToX, d_goToX);
        pidControlY = new SimplePIDControl(p_goToY, i_goToY, d_goToY);
        yellowSampleDrivePowerX = 0.0;
        yellowSampleDrivePowerY = 0.0;
        yellowSampleAveragedDrivePowerX = 0.0;
        yellowSampleAveragedDrivePowerY = 0.0;
        nWhichLimelightPipeline = 0; // 0 is python, 1 is not python
        nNumberLimelightPipelines = 2; // one python yellow and one not python yellow
        headLight = hardwareMap.get(Servo.class, "headLight");
        colorLight = hardwareMap.get(Servo.class, "colorLight");


        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        colorDetector = new ColorDetector(hardwareMap);

        driveTrain = new MecanumDrive(hardwareMap, initialPose);

        averageFilterX = new AverageFilter(3, maxDiffPower);
        averageFilterY = new AverageFilter(3, maxDiffPower);

        armIncrement = 50;
        slideIncrement = 200;

        isDeliveringToBasket = false;
    }
    public void teleInit()
    {
        arm.init();
        arm.claw.start(); // moves to the claw pickup position
    }
    public void autoInit()
    {
        arm.init();
        arm.claw.init(); // keeps it curled so it fits in the 18 x 18 box
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.pipelineSwitch(1); // 0 is python, 1 is not
        limelight.start(); // This tells Limelight to start looking!
    }

    /**
    /**
     * expDrive() sets up exponential driving for teleOp.
     */
    double joyDead = 0.01;         // joystick range in which movement is considered accidental
    double motorMin = 0.05;         // minimum drive motor power

    public double expDrive(double joyVal, double maxVel) {
        if (Math.abs(joyVal) > joyDead) {
            double mOut = ((joyVal * joyVal) * (maxVel - motorMin)) / (1.0 - joyDead);
            double finalOutput = Math.copySign(mOut, joyVal);
            return finalOutput;
        }
        return 0;
    }

    private double offset = 0;

    public double getHeading() {
        double rawHeading = driveTrain.getRawExternalHeading();
        double heading = rawHeading - offset;
        while (heading < 0)
            heading = heading + (2 * Math.PI);
        while (heading > (2 * Math.PI))
            heading = heading - (2 * Math.PI);
        return heading;
    }

    public double resetHeading(double heading) {
        offset = driveTrain.getRawExternalHeading() + heading;

        return offset;
    }

//    public void updateColorDetectorState()
//    {
//        if(colorDetector.checkRed() == 1)
//        {ledStrip.setRed();}
//
//        else if(colorDetector.checkBlue() == 1)
//        {ledStrip.setBlue();}
//
//        else if (colorDetector.checkYellow() == 1)
//        {ledStrip.setYellow();}
//
//        else {ledStrip.setBlack();}
//
//        if (colorDetector.isAbove() == true)
//        {ledStrip.setAboveTrue();}
//    }

    public SampleColor updateColorDetectorState()
    {
        SampleColor returnColor = SampleColor.NA;
        if (colorDetector.isAbove() == true) {
            if (colorDetector.checkRed() == 1) {
                returnColor = SampleColor.RED;
            } else if (colorDetector.checkBlue() == 1) {
                returnColor = SampleColor.BLUE;
            } else if (colorDetector.checkYellow() == 1) {
                returnColor = SampleColor.YELLOW;
            } else {
                returnColor = SampleColor.NA;
            }
        }

        return returnColor;
    }

    public double getRightClawPosition()
    {
        return arm.getRightClawPosition();
    }

    public double getLeftClawPosition()
    {
        return arm.getLeftClawPosition();
    }




    //////////////////////////////////////////////////////////////////////
    // Experimenting with finding the sample below the color sensor
    //////////////////////////////////////////////////////////////////////

    /**
     * sets all the PID variables, the target position,
     * motor mode, motor speed
     */
    public void prepareGoToPositionDrivePID(/*double targetX, double targetY*/)
    {
        // assigning variables from pid controller to
        // variables of this class
        pidControlX.p = p_goToX;
        pidControlX.i = i_goToX;
        pidControlX.d = d_goToX;

        pidControlY.p = p_goToY;
        pidControlY.i = i_goToY;
        pidControlY.d = d_goToY;

        // assign parameter of target to targetPosition of this class
//        targetPositionX = targetX;
        pidControlX.setTargetValue(targetPositionX);

//        targetPositionY = targetY;
        pidControlY.setTargetValue(targetPositionY);

    }

    public void setLimeLightPipeLine(int nWhich) {
        if (nWhich < nNumberLimelightPipelines)
        {
            nWhichLimelightPipeline = nWhich;
            limelight.pipelineSwitch(nWhichLimelightPipeline);
        }
    }
    public void incrementHeadLightUp(){
        headLight.setPosition(headLight.getPosition() + 0.05);
    }
    public void incrementHeadLightDown(){
        headLight.setPosition(0);
    }
    public void incrementColorLightUp(){
        headLight.setPosition(0);
    }
    public void incrementColorLightDown(){
        headLight.setPosition(0);
    }


    /**
     * updateGoToPositionPID uses the PID controller to move the robot
     * to the position for the sample based on the limelight
     *
     * @return
     */

    public void updateGoToPositionDrivePID(boolean useFilteredValue, Telemetry telemetry)
    {
        double actualPositionX = 0.0;
        double actualPositionY = 0.0;
        double validContourFound = 0.0;
        double[] pythonOutputs;
        LLResult result = limelight.getLatestResult();
        double tX = result.getTx();
        double tY = result.getTy();

        pythonOutputs = result.getPythonOutput();
        if (pythonOutputs != null && pythonOutputs.length > 0) {

            // first output is if valid contour found
            validContourFound = pythonOutputs[0];

            // second and third output are the x and y location in image coordinates
            actualPositionX = pythonOutputs[1];
            actualPositionY = pythonOutputs[2];

//            // look at all the elements in the python output array in telemetry
//            telemetry.addData("0", pythonOutputs[0]);
//            dashboardTelemetry.addData("0", pythonOutputs[0]);
//            telemetry.addData("1", pythonOutputs[1]);
//            dashboardTelemetry.addData("1", pythonOutputs[1]);
//            telemetry.addData("2", pythonOutputs[2]);
//            dashboardTelemetry.addData("2", pythonOutputs[2]);
//            telemetry.addData("3", pythonOutputs[3]);
//            dashboardTelemetry.addData("3", pythonOutputs[3]);
//            telemetry.addData("4", pythonOutputs[4]);
//            dashboardTelemetry.addData("4", pythonOutputs[4]);
        } else {
            // TODO what do we do if there isn't a valid output from the limelight?
            telemetry.addData("Limelight", "No Targets");
        }

        //actualPositionY = limelight.getLatestResult().getTy();
        telemetry.addData("Which Pipeline", result.getPipelineIndex());
        dashboardTelemetry.addData("Which Pipeline", result.getPipelineIndex());
        telemetry.addData("Actual X Position", actualPositionX);
        telemetry.addData("Actual Y Position", actualPositionY);
        telemetry.addData("tX Value", tX);
        telemetry.addData("tY value", tY);

        // calculate the power for the X pid using the actual position and the maximum tics
        yellowSampleDrivePowerX = pidControlX.update(actualPositionX, feedForward) / (maxTicsX);
        //TODO ADD CLAMP TO DRIVE X
      //  yellowSampleDrivePowerX = pidControlX.clamp(yellowSampleDrivePowerX, 0,)

        // calculate the power for the Y pid using the actual position and the maximum tics
        yellowSampleDrivePowerY = pidControlY.update(actualPositionY, feedForward) / (maxTicsY);

        // now use this power to compute the averaged version for x
        averageFilterX.setNewValue(yellowSampleDrivePowerX);
        yellowSampleAveragedDrivePowerX = averageFilterX.getCurrentAverage();

        // now use this power to compute the averaged version for y
        averageFilterY.setNewValue(yellowSampleDrivePowerY);
        yellowSampleAveragedDrivePowerY = averageFilterY.getCurrentAverage();

        if (useFilteredValue)
        {
            pidPowerSampleX = yellowSampleAveragedDrivePowerX;
            pidPowerSampleY = yellowSampleAveragedDrivePowerY;
        }
        else
        {
            pidPowerSampleX = yellowSampleDrivePowerX;
            pidPowerSampleY = yellowSampleDrivePowerY;
        }

        // calculate the error using the pid x controller measured error function
        double errorX = Math.abs(pidControlX.measuredError(actualPositionX));

        // calculate the error using the pid x controller measured error function
        double errorY = Math.abs(pidControlY.measuredError(actualPositionY));

        // robot logging
        RobotLog.d("WGW8620 - updateGoToPositionDrivePID " + "errorX = " + errorX +
                " errorY = " + errorY +
                " pidPowerX =" + pidPowerSampleX +
                " pidPowerY =" + pidPowerSampleY +
                " Calculated power for x = " + yellowSampleDrivePowerX +
                " Calculated power for y = " + yellowSampleDrivePowerY +
//                " Averaged power for x = " + yellowSampleAveragedDrivePowerX +
//                " Averaged power for y = " + yellowSampleAveragedDrivePowerY +
                " limelight x value = " + actualPositionX +
                " limelight y value = " + actualPositionY +
                " Current time " + System.currentTimeMillis());

        // show how old the data is
        long staleness = result.getStaleness();
        //dashboardTelemetry.addData("staleness", staleness);

        // dashboard telemetry for sample x driving
        //dashboardTelemetry.addData("target X", targetPositionX);
        dashboardTelemetry.addData("actual tx position python", actualPositionX);
        //dashboardTelemetry.addData("actual tx position", tX);
        //dashboardTelemetry.addData("error X ", errorX);
        //dashboardTelemetry.addData("calculated power X", yellowSampleDrivePowerX);
        //dashboardTelemetry.addData("averaged power X", yellowSampleAveragedDrivePowerX);

        // dashboard telemetry for sample y driving
        //dashboardTelemetry.addData("target Y", targetPositionY);
        dashboardTelemetry.addData("actual ty position python", actualPositionY);
        //dashboardTelemetry.addData("actual ty position", tY);
        //dashboardTelemetry.addData("error Y", errorY);
        //dashboardTelemetry.addData("calculated power Y", yellowSampleDrivePowerX);
        //dashboardTelemetry.addData("averaged power Y", yellowSampleAveragedDrivePowerX);

        dashboardTelemetry.addData("pid power sample y", pidPowerSampleY);
        dashboardTelemetry.addData("pid power sample x", pidPowerSampleX);

        dashboardTelemetry.update();
    }


    public boolean sampleAutoTargeting(@NonNull Telemetry telemetry) {
        updateGoToPositionDrivePID(false, telemetry);
        RobotLog.d("WGW8620 - sampleAutoTargeting - First Update Call Before While Loop" );
        while ((Math.abs(pidPowerSampleY) > 0.08) || (Math.abs(pidPowerSampleX) > 0.08)) {
            updateGoToPositionDrivePID(false, telemetry);

            RobotLog.d("WGW8620 - sampleAutoTargeting -  Update Call In While Loop" );

// ******** BEGIN - base drive wheel control  *****
            // Run robot in POV mode - pushing joystick away makes robot move away regardless of rotation
            // note: The joystick goes negative when pushed forwards, so negate it
            //  Find robot's field axes in relation to joystick axes
            //double thetaRadians    = 0;  // temp use 0 for heading
            double thetaRadians = getHeading();  // get direction is robot facing
            double joyStick_x_axis =0;
//        double joyStick_x_axis = -ferb.pidPowerSampleX;
//        double joyStick_y_axis = ferb.expDrive(gamepad1.left_stick_y, maxVel);
            double joyStick_y_axis = -pidPowerSampleY;

//        double  joystick_turn  = ferb.expDrive(gamepad1.right_stick_x, maxVel*2/3);
            double joystick_turn  = -pidPowerSampleX;
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
//            driveTrain.leftFront.setPower(frontLeft);
//            driveTrain.leftBack.setPower(backLeft);
//            driveTrain.rightFront.setPower(frontRight);
//            driveTrain.rightBack.setPower(backRight);


            telemetry.addData("PID power x", pidPowerSampleX);
            telemetry.addData("PID Power y", pidPowerSampleY);

            //telemetry.addData("heading", ferb.getHeading());

            //telemetry.addData("loopTime", loop);
            // telemetry.addData("ourTime", loopTimer.milliseconds());

            telemetry.update();

        }
        RobotLog.d("WGW8620 - sampleAutoTargeting -  While Loop Finished" );

        return true;
    }
}