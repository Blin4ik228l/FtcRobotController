package org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Extenders.Extenders2;


import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Config.MainFile;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Wrappers.HardwareBuilder;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.DeviceTree.DeviceCollector;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.DeviceTree.DeviceManager;

public abstract class ExecutableModule extends DeviceManager {
    public ExecutableModule(MainFile mainFile, String searchingDevice){
        super(mainFile, searchingDevice);
    }
    public void execute(Double...args){
        if(!isInitialized || isInterrupted) return;
        else executeExt(args);
    };
    protected abstract void executeExt(Double...args);
}
