package org.firstinspires.ftc.teamcode.MainParts.Modules.Extenders;

import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.Wrappers.Examples.ColorSensorWrapper;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.Wrappers.Examples.MotorWrapper;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.Wrappers.Examples.ServoMotorWrapper;
import org.firstinspires.ftc.teamcode.MainParts.Modules.DeviceTree.DeviceCollector;

public abstract class MainModule extends DeviceCollector {
    public void createServoWrapperUtils(){
        servosCollector = new ServoMotorWrapper.InnerCollector();
    }

    public void createColorWrapperUtils(){
        colorsCollector = new ColorSensorWrapper.InnerCollector();
    }
    public void createMotorWrapperUtils(){
        motorsCollector = new MotorWrapper.InnerCollector();
    }

    @Override
    protected void sayLastWords() {
        telemetry.addLine("============");
        telemetry.addLine();
    }

    @Override
    public void sayModuleName() {
        telemetry.addLine( "===" + this.getClass().getSimpleName().toUpperCase() + "===");

    }
}
