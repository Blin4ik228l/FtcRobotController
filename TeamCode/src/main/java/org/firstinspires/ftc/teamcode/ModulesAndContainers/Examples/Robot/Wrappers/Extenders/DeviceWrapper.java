package org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Wrappers.Extenders;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Module;

public abstract class DeviceWrapper extends Module {
    protected String searchingDevice;
    public DeviceWrapper(OpMode op) {
        super(op);
    }

    public String getSearchingDevice() {
        return searchingDevice;
    }

    @Override
    public void sayInited() {
        if (!isInitialized) telemetry.addLine("Bad initialize bc " + " " + searchingDevice + " " + " not found/attached");
        else telemetry.addLine("scfly inited" + " " + searchingDevice);
    }
    @Override
    public void showData() {

    }
}
