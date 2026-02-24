package org.firstinspires.ftc.teamcode.ModulesAndContainers.Containers;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Players.Pl2.AutoPlayerClass2;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Module;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Extenders.UpdatableModule;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;
import java.util.Locale;

public class UpdatableContainer extends Container {
    public ArrayList<UpdatableModule> modules;
    public UpdatableContainer(ModulesCollector modulesCollector){
        super(modulesCollector);

        for (Module module: allModules) {
            if (module instanceof UpdatableModule){
                modules.add((UpdatableModule) module);
            }
        }
    }

    @Override
    public void run() {
        while (running && !Thread.currentThread().isInterrupted())
        {
            long start = System.nanoTime();
            update();

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

    public void update() {
//        for (UpdatableModule module : modules){
//            if(module.isInizialized) {module.update();
//            module.increaseIteration();}
//        };
//
//        if (modulesCollector.teleOpModernized != null){
//            modulesCollector.teleOpModernized.extUpdate();
//        }
    }
}
