package org.firstinspires.ftc.teamcode.MainParts.Modules.Extenders;

public abstract class UpdatableCollector extends MainModule {
    public UpdatableCollector(boolean isThisExecutingOtherModules) {
        super(isThisExecutingOtherModules);
    }
    protected abstract void updateExt();
    protected int iterationCount = 1;

    public void update(int curIterations, int targIterations) {
        iterationCount = curIterations;
        if(!isInitialized || curIterations % targIterations != 0) return;
        else updateExt();
    }
}
