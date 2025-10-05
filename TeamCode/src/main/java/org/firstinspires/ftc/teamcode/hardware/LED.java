package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.ColorDetector;

public class LED {
    RevBlinkinLedDriver blinkinLedDriver;



    public LED(HardwareMap hardwareMap) {
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "LED");
    }

    public void setRed() {
        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
    }

    public void setBlue() {
        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
    }

    public void setYellow() {
        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GOLD);
    }

    public void setBlack() {
        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
    }

    public void setHotPink() {
        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
    }
    public void setGreen() {
        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
    }

    public void setAboveTrue() {
        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.CONFETTI);
    }
    public void updateLED(Robot.SampleColor sampleColor) {
        if (sampleColor == Robot.SampleColor.RED) {
            blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        }
        else if (sampleColor == Robot.SampleColor.BLUE) {
            blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
        }

        else if (sampleColor == Robot.SampleColor.YELLOW) {
            blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GOLD);
        }

        else if (sampleColor == Robot.SampleColor.ABOVE) {
            blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
        }

        else
        {
            blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
        }

    }
}