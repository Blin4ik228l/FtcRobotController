package org.firstinspires.ftc.teamcode.MainParts.Modules.Extenders;

public abstract class UpdatableCollector extends MainModule {
    public UpdatableCollector(boolean isThisExecutingOtherModules) {
        super(isThisExecutingOtherModules);
    }
    protected abstract void updateExt();
    protected int iterationCount = 1;
    protected boolean isNeedUpdate = true;
    public void update(int curIterations, int targIterations) {
        iterationCount = curIterations;
        if(!isInitialized || !isNeedUpdate || curIterations % targIterations != 0) return;
        else updateExt();
    }
    public void stopUpdate(){
        isNeedUpdate = false;
    }
    public void continueUpdate(){
        isNeedUpdate = true;
    }
}
