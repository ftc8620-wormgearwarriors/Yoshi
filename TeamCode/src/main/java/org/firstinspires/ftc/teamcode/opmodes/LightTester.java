package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;


import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.hardware.Robot;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="LightTester")
public class LightTester extends OpMode {

    Robot ferb;

    Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));

    ElapsedTime loopTimer = new ElapsedTime();

    boolean debugMode = false;

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

        if (gamepad1.a){
            ferb.headLight.setPosition(ferb.headLight.getPosition() + 0.05);
        }

        if (gamepad1.b){
            ferb.headLight.setPosition(ferb.headLight.getPosition() - 0.05);
        }

        if (gamepad1.x){
            ferb.colorLight.setPosition(ferb.colorLight.getPosition() + 0.05);
        }

        if (gamepad1.y){
            ferb.colorLight.setPosition(ferb.colorLight.getPosition() - 0.05);
        }
        // these messages are displayed all the time, not just during debug mode
        telemetry.addData("Color Light", ferb.colorLight.getPosition());
        telemetry.addData("Head Light", ferb.headLight.getPosition());
        telemetry.update();
    }
}
