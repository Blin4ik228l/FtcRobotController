package org.firstinspires.ftc.teamcode.MainParts.Modules.Extenders;

public abstract class ExecutableCollector extends MainModule {
    public void execute(Double...args){
        if(!isInitialized) return;
        else executeExt(args);
    }
    protected abstract void executeExt(Double... args);

    @Override
    public void sayModuleName() {
        telemetry.addLine( "[" + this.getClass().getSimpleName().toUpperCase() + "]");
    }

    @Override
    protected void sayLastWords() {
        telemetry.addLine("[!!!!!!!!!!!!!!]");
    }
}
