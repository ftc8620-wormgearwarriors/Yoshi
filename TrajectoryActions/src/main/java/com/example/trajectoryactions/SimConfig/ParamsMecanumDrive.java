package com.example.trajectoryactions.SimConfig;

public class ParamsMecanumDrive {        // IMU orientation
//    // TODO: fill in these values based on
//    //   see https://ftc-docs.firstinspires.org/en/latest/programming_resources/imu/imu.html?highlight=imu#physical-hub-mounting
//    public RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection =
//            RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
//    public RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection =
//            RevHubOrientationOnRobot.UsbFacingDirection.UP;

    // drive model parameters
    public double inPerTick = 0.00296843400435984199124204316674;
    public double lateralInPerTick = inPerTick;
    public double trackWidthTicks = 0;

    // feedforward parameters (in tick units)
    public double kS = 0;
    public double kV = 0;
    public double kA = 0;

    // path profile parameters (in inches)
    public double maxWheelVel = 50;
    public double minProfileAccel = -30;
    public double maxProfileAccel = 50;

    // turn profile parameters (in radians)
    public double maxAngVel = Math.PI; // shared with path
    public double maxAngAccel = Math.PI;

    // path controller gains
    public double axialGain = 0.0;
    public double lateralGain = 0.0;
    public double headingGain = 0.0; // shared with turn

    public double axialVelGain = 0.0;
    public double lateralVelGain = 0.0;
    public double headingVelGain = 0.0; // shared with turn

}
