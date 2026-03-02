package org.firstinspires.ftc.teamcode.ModulesAndContainers.Containers;

import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Extenders.Extenders2.ExecutableModule;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Module;

import java.util.ArrayList;

public class ExecutableContainer extends Container {
    public ArrayList<ExecutableModule> modules;
    public ExecutableContainer(ModulesCollector modulesCollector){
        super(modulesCollector);

        for (Module module: allModules) {
            if (module instanceof ExecutableModule){
                modules.add((ExecutableModule) module);
            }
        }
    }

    @Override
    public void run() {
        while (running && !Thread.currentThread().isInterrupted())
        {
            long start = System.nanoTime();
            execute();

            long elapsed = System.nanoTime() - start;
            long sleepTime = 20_000_000 - elapsed; // 20ms в наносекундах

            if (sleepTime > 0) {
                try {
                    Thread.sleep(20);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }
        }
    }

    public void execute() {
//        for (ExecutableModule module : modules){
//            if(module.isInizialized) module.execute();
//        }
//
//        if (modulesCollector.teleOpModernized != null){
//            modulesCollector.teleOpModernized.extExecute();
//        }
    }
}
