package org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.DeviceTree;


import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Config.MainFile;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Wrappers.Examples.MotorWrapper;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Wrappers.Examples.ServoMotorWrapper;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Wrappers.HardwareBuilder;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Module;

import java.util.HashMap;

public abstract class DeviceManager extends Module {
    protected HardwareMap hardwareMap;
    protected String searchingDevice;

    public DeviceManager(MainFile mainFile, String searchingDevice) {
        super(mainFile);
        this.searchingDevice = searchingDevice;
    }

    public abstract void showDataExt();
    public void sayInited() {
        if (!isInitialized) telemetry.addLine("Bad initialize" + " " + searchingDevice);
        else telemetry.addLine("scfly inited" + " " + searchingDevice);
    }
    @Override
    protected void sayModuleName() {
        telemetry.addLine("==="+ searchingDevice +"===");
    }
    @Override
    public void showData() {
        if(!isInitialized) telemetry.addLine( searchingDevice + " " + "Not Found/Attached");
        else showDataExt();
    }

    public abstract static class Builder<T> extends HardwareBuilder {
        protected T wrapper;
        @Override
        public abstract Builder initialize(MainFile mainFile, String searchingDevice);
        public abstract Builder setFields(Double...args);

        public T get(){
            return wrapper;
        }
    }
    public static abstract class CollectorBuilder<T>{
        protected HashMap<String, T> wrappers = new HashMap<>();

        public abstract CollectorBuilder add(T wrapper);

        public T get(String deviceName){
            return wrappers.get(deviceName);
        };

        public abstract void showData();
    }
}
