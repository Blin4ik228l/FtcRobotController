package org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Extenders;

import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Config.MainFile;

public abstract class ExecutableCollector extends MainModule {
    public ExecutableCollector(MainFile mainFile) {
        super(mainFile);
    }

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
