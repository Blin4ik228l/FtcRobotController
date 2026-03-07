package org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Extenders;

import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Config.MainFile;

public abstract class UpdatableCollector extends MainModule {
    public UpdatableCollector(MainFile mainFile) {
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
    @Override
    public void sayModuleName() {
        telemetry.addLine( "[" + this.getClass().getSimpleName().toUpperCase() + "]");
    }
    public void update() {
        if(!isInitialized || isInterrupted || iterationCount % updateCount != 0) return;
        else updateExt();
    }

    @Override
    protected void sayLastWords() {
        telemetry.addLine("[??????????????]");
    }
}
