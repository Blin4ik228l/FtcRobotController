package org.firstinspires.ftc.teamcode.MainParts.Modules.DeviceTree;

import org.firstinspires.ftc.teamcode.MainParts.Examples.Wrappers.Examples.ColorSensorWrapper;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Wrappers.Examples.MotorWrapper;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Wrappers.Examples.ServoMotorWrapper;

public abstract class DeviceCollector extends DevicesBuilder {
    protected ServoMotorWrapper.InnerCollector servosCollector;
    protected ColorSensorWrapper.InnerCollector colorsCollector;
    protected MotorWrapper.InnerCollector motorsCollector;
}
