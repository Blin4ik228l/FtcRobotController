package org.firstinspires.ftc.teamcode.MainParts.Modules.DeviceTree;

import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.Wrappers.Examples.ColorSensorWrapper;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.Wrappers.Examples.MotorWrapper;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.Wrappers.Examples.ServoMotorWrapper;

public abstract class DevicesBuilder extends ListOfAllDevices{
    protected ServoMotorWrapper.InnerBuilder servoBuilder = new ServoMotorWrapper.InnerBuilder();
    protected ColorSensorWrapper.InnerBuilder colorBuilder = new ColorSensorWrapper.InnerBuilder();
    protected MotorWrapper.InnerBuilder motorBuilder = new MotorWrapper.InnerBuilder();
}
