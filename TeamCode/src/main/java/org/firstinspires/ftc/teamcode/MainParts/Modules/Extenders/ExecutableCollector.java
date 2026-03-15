package org.firstinspires.ftc.teamcode.MainParts.Modules.Extenders;

public abstract class ExecutableCollector extends MainModule {
    public ExecutableCollector(boolean isThisExecutingOtherModules) {
        super(isThisExecutingOtherModules);
    }

    public void execute(Double...args){
        if (isInitialized) executeExt(args);
    }
    protected abstract void executeExt(Double... args);
}
