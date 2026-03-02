package org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.DeviceTree;

import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Config.MainFile;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Wrappers.Examples.ColorSensorWrapper;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Wrappers.Examples.MotorWrapper;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Wrappers.Examples.ServoMotorWrapper;

public abstract class DeviceCollector extends DevicesBuilder {
    protected ServoMotorWrapper.InnerCollector servosCollector;
    protected ColorSensorWrapper.InnerCollector colorsCollector;
    protected MotorWrapper.InnerCollector motorsCollector;
    public DeviceCollector(MainFile mainFile) {
        super(mainFile);
    }
}
