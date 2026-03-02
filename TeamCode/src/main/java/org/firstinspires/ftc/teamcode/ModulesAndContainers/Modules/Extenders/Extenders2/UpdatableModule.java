package org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Extenders.Extenders2;

import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Config.MainFile;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.DeviceTree.DeviceManager;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.DeviceTree.DevicesBuilder;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Module;

public abstract class UpdatableModule extends DeviceManager {
    public UpdatableModule(MainFile mainFile, String searchingDevice) {
        super(mainFile, searchingDevice);
    }
    public void update() {
        if(!isInitialized || isInterrupted) return;
        else updateExt();
    }
    protected abstract void updateExt();
}
