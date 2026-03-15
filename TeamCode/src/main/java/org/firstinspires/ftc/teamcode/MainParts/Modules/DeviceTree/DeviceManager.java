package org.firstinspires.ftc.teamcode.MainParts.Modules.DeviceTree;


import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.Config.MainFile;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.Wrappers.HardwareBuilder;
import org.firstinspires.ftc.teamcode.MainParts.Modules.Module;

import java.util.HashMap;

public abstract class  DeviceManager extends Module {
    protected HardwareMap hardwareMap;
    protected String searchingDevice;

    public DeviceManager(String searchingDevice) {
        this.hardwareMap = MainFile.op.hardwareMap;
        this.searchingDevice = searchingDevice;
    }

    protected abstract void showDataExt();
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
    public void showData() {
        sayModuleName();
        if (!isInitialized) {
            sayBadInit();
        }else showDataExt();
    }

    @Override
    protected void sayLastWords() {

    }

    public abstract static class Builder<T> extends HardwareBuilder {
        protected T wrapper;
        @Override
        public abstract Builder initialize(String searchingDevice);
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
