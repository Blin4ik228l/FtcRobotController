package org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.DeviceTree;


import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Config.MainFile;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Wrappers.HardwareBuilder;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Module;

import java.util.HashMap;

public abstract class DeviceManager extends Module {
    protected HardwareMap hardwareMap;
    protected String searchingDevice;

    public DeviceManager(MainFile mainFile, String searchingDevice) {
        super(mainFile);
        this.hardwareMap = mainFile.op.hardwareMap;
        this.searchingDevice = searchingDevice;
    }

    public abstract void showDataExt();
    public void sayInited() {
        if (!isInitialized) telemetry.addLine("Bad initialize" + " " + searchingDevice);
        else telemetry.addLine("scfly inited" + " " + searchingDevice);
    }
    @Override
    public void sayModuleName() {
        telemetry.addLine("|/-"+ searchingDevice +"-|/");
    }

    public void sayBadInit(){
        telemetry.addLine("Not Found/Attached");
    }

    @Override
    protected void sayLastWords() {

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
