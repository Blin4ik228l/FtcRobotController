package org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Extenders;

import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Players.PL0.ProgramState;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Config.MainFile;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.DeviceTree.DeviceCollector;

public abstract class UpdatingModule extends MainModule {
    public UpdatingModule(MainFile mainFile) {
        super(mainFile);
    }
    protected abstract void updateExt();

    protected int updateCount = 1;
    protected int iterationCount = 1;
    public void setUpdateCount(int count){
        this.updateCount = count;
    }
    public void setCurIterations(int count){
        iterationCount = count;
    }
    public void update() {
        if(!isInitialized || isInterrupted || iterationCount % updateCount != 0) return;
        else updateExt();
    }
}
