package org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Extenders;

import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Config.MainFile;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Wrappers.Examples.ColorSensorWrapper;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Wrappers.Examples.MotorWrapper;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Wrappers.Examples.ServoMotorWrapper;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.DeviceTree.DeviceCollector;

public abstract class MainModule extends DeviceCollector {
    public MainModule(MainFile mainFile) {
        super(mainFile);
    }
    public void createServoWrapperUtils(){
        servoBuilder = new ServoMotorWrapper.InnerBuilder();
        servosCollector = new ServoMotorWrapper.InnerCollector();
    }

    public void createColorWrapperUtils(){
        colorBuilder = new ColorSensorWrapper.InnerBuilder();
        colorsCollector = new ColorSensorWrapper.InnerCollector();
    }
    public void createMotorWrapperUtils(){
        motorBuilder = new MotorWrapper.InnerBuilder();
        motorsCollector = new MotorWrapper.InnerCollector();
    }

    @Override
    public void sayModuleName() {
        telemetry.addLine( "===" + this.getClass().getSimpleName().toUpperCase() + "===");
    }
}
