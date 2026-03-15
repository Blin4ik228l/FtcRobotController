package org.firstinspires.ftc.teamcode.MainParts.Modules.DeviceTree;

import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.Wrappers.Examples.ColorSensorWrapper;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.Wrappers.Examples.MotorWrapper;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.Wrappers.Examples.ServoMotorWrapper;

public abstract class DeviceCollector extends DevicesBuilder {
    protected ServoMotorWrapper.InnerCollector servosCollector;
    protected ColorSensorWrapper.InnerCollector colorsCollector;
    protected MotorWrapper.InnerCollector motorsCollector;
}
