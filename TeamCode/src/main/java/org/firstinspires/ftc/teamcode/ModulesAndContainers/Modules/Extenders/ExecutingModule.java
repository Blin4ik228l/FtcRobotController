package org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Extenders;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Players.PL0.ProgramState;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Config.MainFile;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.DeviceTree.DeviceCollector;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Extenders.Extenders2.ExecutableModule;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Module;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Deque;

public abstract class ExecutingModule extends MainModule {
    public ExecutingModule(MainFile mainFile) {
        super(mainFile);
    }
    public void execute(Double...args){
        if(!isInitialized || isInterrupted) return;
        else executeExt(args);
    };
    protected abstract void executeExt(Double... args);
}
