package org.firstinspires.ftc.teamcode.MainParts.Modules.Extenders;

import org.firstinspires.ftc.teamcode.MainParts.Examples.Wrappers.Examples.ColorSensorWrapper;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Wrappers.Examples.MotorWrapper;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Wrappers.Examples.ServoMotorWrapper;
import org.firstinspires.ftc.teamcode.MainParts.Modules.DeviceTree.DeviceCollector;

public abstract class MainModule extends DeviceCollector {
    public MainModule(boolean isThisExecutingOtherModules){
        this.isThisExecutingOtherModules = isThisExecutingOtherModules;
    }
    protected boolean isThisExecutingOtherModules;
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
    public void sayModuleName() {
        if(isThisExecutingOtherModules) telemetry.addLine( "[" + this.getClass().getSimpleName().toUpperCase() + "-" + "contains" +"]");
        else telemetry.addLine("-" + this.getClass().getSimpleName().toUpperCase() + "-");
    }

    @Override
    protected void sayLastWords() {
        if(isThisExecutingOtherModules) telemetry.addLine("[" + this.getClass().getSimpleName().toUpperCase() + " " + "ended" + "]");
    }
}
