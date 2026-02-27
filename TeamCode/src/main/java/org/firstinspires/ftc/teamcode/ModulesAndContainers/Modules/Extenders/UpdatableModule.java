package org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Extenders;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Module;

public abstract class UpdatableModule extends Module {
    public UpdatableModule(OpMode op) {
        super(op);
    }

    public void safeUpdate(){
        if(isInitialized) update();
    }
    protected abstract void update();
}
