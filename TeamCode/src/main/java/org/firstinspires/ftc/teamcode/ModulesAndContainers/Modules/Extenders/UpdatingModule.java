package org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Extenders;

import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Players.PL0.ProgramState;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Config.MainFile;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.DeviceTree.DeviceCollector;

public abstract class UpdatingModule extends MainModule {
    public UpdatingModule(MainFile mainFile) {
        super(mainFile);
    }
    protected abstract void updateExt();

    public void update() {
        if(!isInitialized || isInterrupted) return;
        else updateExt();
    }
}
