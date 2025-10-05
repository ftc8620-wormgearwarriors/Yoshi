package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.LED;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ColorDetector {

    private HardwareMap m_hwMap = null;


    // Define a variable for our color sensor
    public ColorSensor colorSensor = null;
    public LED ledStrip;

    ;
    private Robot.SampleColor colorHolding;
    private int redThresh;
    private int greenThresh;
    private int blueThresh;

    // Red is zero, Yellow is 1, Blue is 2
    private int currentColor;

    public ColorDetector(HardwareMap hwMap)
    // Get the color sensor from hardwareMap

    {
        m_hwMap = hwMap;

        colorSensor = m_hwMap.get(ColorSensor.class, "colorSensor");
        currentColor = -1;
        blueThresh = 600;
        redThresh = 600;
        greenThresh = 600;

        ledStrip = new LED(hwMap);

    }

    public int getRed() {
        return redThresh;
    }

    public int getBlue() {
        return blueThresh;
    }

    public int getGreen() {
        return greenThresh;
    }

    public void setRed(int red) {
        this.redThresh = red;

    }

    public void setBlue(int blue) {
        this.blueThresh = blue;
    }

    public void setGreen(int green) {
        this.greenThresh = green;
    }

    public int checkRed() {
        currentColor = 0;
        if(( colorSensor.red() > redThresh) &&
                (colorSensor.green() < greenThresh) &&
                (colorSensor.blue() < blueThresh)){
            currentColor = 1;
            //also check blue and green is under threshold



        }


        return currentColor;
    }


    public int checkYellow() {
        currentColor = 0;
        if (( colorSensor.red() > redThresh) &&
                (colorSensor.green() > greenThresh) &&
                (colorSensor.blue() < blueThresh)) {
            currentColor = 1;
            //also check blue is under threshold

        }

        return currentColor;

    }
    public int checkBlue() {
        currentColor = 0;
        if(( colorSensor.red() < redThresh) &&
                (colorSensor.green() < greenThresh) &&
                (colorSensor.blue() > blueThresh)) {
            currentColor = 1;
            //also check red and green are under threshold

        }

        return currentColor;
    }

    public Robot.SampleColor colorIs() {
        if (checkRed() == 1 && checkBlue() == 0 && checkYellow() == 0)
        {colorHolding = Robot.SampleColor.RED;
        }


        else if(checkRed() == 0 && checkBlue() == 1 && checkYellow() == 0)
        {colorHolding = Robot.SampleColor.BLUE;}


        else if(checkRed() == 0 && checkBlue() == 0 && checkYellow() == 1)
        {colorHolding = Robot.SampleColor.YELLOW;}

        else if (isAbove())
        {colorHolding = Robot.SampleColor.ABOVE;}


        else {colorHolding = Robot.SampleColor.NA;}

        return colorHolding;
    }

    public boolean isAbove(){
        boolean isAbove = false;
        if (distance() <= 7.5 && distance() > 1.5)
        {
            isAbove = true;
        }

        return isAbove;
    }

    public int blue (){
        return colorSensor.blue();
    }
    public int green () {
        return colorSensor.green();
    }
    public int red () {
        return colorSensor.red();
    }

    public double distance(){
        return ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM);
    }

}
