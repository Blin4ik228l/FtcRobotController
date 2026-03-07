package org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.DeviceTree;

import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Config.ControlHubDevices;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Config.ExpansionHubDevices;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Config.MainFile;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Wrappers.Examples.ColorSensorWrapper;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Wrappers.Examples.MotorWrapper;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Wrappers.Examples.ServoMotorWrapper;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Wrappers.HardwareBuilder;

public abstract class DevicesBuilder extends ListOfAllDevices{
    protected ServoMotorWrapper.InnerBuilder servoBuilder = new ServoMotorWrapper.InnerBuilder();
    protected ColorSensorWrapper.InnerBuilder colorBuilder = new ColorSensorWrapper.InnerBuilder();
    protected MotorWrapper.InnerBuilder motorBuilder = new MotorWrapper.InnerBuilder();
    public DevicesBuilder(MainFile mainFile) {
        super(mainFile);
    }


}
