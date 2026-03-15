package org.firstinspires.ftc.teamcode.MainParts.Modules.Extenders.Extenders2;


import org.firstinspires.ftc.teamcode.MainParts.Modules.DeviceTree.DeviceManager;

public abstract class ExecutableModule extends DeviceManager {
    public ExecutableModule(String searchingDevice){
        super(searchingDevice);
    }
    public void execute(Double...args){
        if(!isInitialized) return;
        else executeExt(args);
    };
    protected abstract void executeExt(Double...args);

    @Override
    protected void sayLastWords() {
        super.sayLastWords();
    }
}
