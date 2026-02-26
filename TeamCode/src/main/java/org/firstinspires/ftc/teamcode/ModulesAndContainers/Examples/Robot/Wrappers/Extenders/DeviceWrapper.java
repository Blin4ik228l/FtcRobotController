package org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Wrappers.Extenders;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Module;

public abstract class DeviceWrapper extends Module {
    protected String deviceName;
    public DeviceWrapper(OpMode op) {
        super(op);
    }
    @Override
    public void sayInited() {
        if (!isInitialized) telemetry.addLine("Bad initialize bc " + " " + deviceName + " " + " not found/attached");
        else telemetry.addLine("scfly inited" + " " + deviceName);
    }
    @Override
    public void showData() {

    }
}
