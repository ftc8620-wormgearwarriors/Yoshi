package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;


import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.hardware.Robot;

import java.util.ArrayList;
import java.util.List;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOpActions")
public class TeleopActions extends OpMode {
    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();

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
        telemetry.addData(">", "DO NOT START YET");
        telemetry.update();

        ferb = new Robot(hardwareMap, startPose);

        //it is done initializing ready to start!
        telemetry.addData(">", "READY TO START!");
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

    public void loop() {

        TelemetryPacket packet = new TelemetryPacket();

        // updated based on gamepads

        // update running actions
        List<Action> newActions = new ArrayList<>();
        for (Action action : runningActions) {
            action.preview(packet.fieldOverlay());
            if (action.run(packet)) {
                newActions.add(action);
            }
        }
        runningActions = newActions;

        dash.sendTelemetryPacket(packet);

        if(gamepad1.a){
            runningActions.add((new SequentialAction(
                    new InstantAction(() -> ferb.arm.claw.openClaw()),
                    new InstantAction(() -> ferb.arm.claw.closeClaw())
            )));
        }

        if(gamepad1.b){
            runningActions.add((new SequentialAction(
                    new InstantAction(() -> ferb.arm.armTeleDriving()),
                    new InstantAction(() -> ferb.arm.claw.setWristSamplePrePickup()),
                    new InstantAction(() -> ferb.arm.claw.openClaw())
            )));
        }


        if(gamepad1.x){
            runningActions.add((new SequentialAction(
                    new InstantAction(() -> ferb.arm.armTelePickup()),
                    new InstantAction(() -> ferb.arm.claw.setWristPickup()),
                    new SleepAction(0.75),
                    new InstantAction(() -> ferb.arm.claw.closeClaw()),
                    new SleepAction(0.75),
                    new ParallelAction(
                            ferb.arm.armToHighBasket(),
                            ferb.arm.slideToHighBasket()
                            ),
                    new InstantAction(() -> ferb.arm.claw.openClaw())
            )));


        }
    }



}
