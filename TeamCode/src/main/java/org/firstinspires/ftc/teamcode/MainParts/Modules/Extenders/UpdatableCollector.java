package org.firstinspires.ftc.teamcode.MainParts.Modules.Extenders;

public abstract class UpdatableCollector extends MainModule {
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
    public void update(int curIterations, int targIterations) {
        iterationCount = curIterations;
        if(!isInitialized || isInterrupted || curIterations % targIterations != 0) return;
        else updateExt();
    }

    @Override
    protected void sayLastWords() {
        telemetry.addLine("[??????????????]");
    }
}
