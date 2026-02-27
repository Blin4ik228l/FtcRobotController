package org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Wrappers.Extenders;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Extenders.UpdatableModule;

public abstract class DeviceUpdaterWrapper extends DeviceWrapper {
    public DeviceUpdaterWrapper(OpMode op) {
        super(op);
    }

    public void update() {
        if(!isInitialized) return;
        else updateExt();
    }
    protected abstract void updateExt();
}
