package org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Config;

public class ControlHubDevices extends DevicesConfig{
    public ControlHubDevices(){
        new Builder()
                .setMotorNames("")
                .setServoNames("")
                .setI2CDeviceNames("")
                .setDigDeviceNames("")
                .setAnalogDeviceNames("");
    }
}
