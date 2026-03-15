package org.firstinspires.ftc.teamcode.MainParts.Containers;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Players.Pl2.AutoPlayerClass2;
import org.firstinspires.ftc.teamcode.MainParts.Modules.Module;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;
import java.util.Locale;

public class TelemetryContainer extends Container {
    public ArrayList<Module[]> modules;
    public TelemetryContainer(ModulesCollector modulesCollector) {
        super(modulesCollector);
        //Показать всё
        modules.add(allModules);

        //Показать только робота
        modules.add(new Module[]{
                modulesCollector.robot
        });

        //Вывод телеги
        modules.add(new Module[]{
                modulesCollector.robot.drivetrain
        });
        //Вывод системы подачи
        modules.add(new Module[]{
                modulesCollector.robot.collector
        });

        //вывод одометрии
        modules.add(new Module[]{
                modulesCollector.robot.drivetrain.navigationSystem
        });

        //вывод игроков
        modules.add(new Module[]{
                modulesCollector.semiAutoPlayerClass1, modulesCollector.autoPlayerClass2
        });

        //Вывод джойстиков
        modules.add(new Module[]{
                modulesCollector.semiAutoPlayerClass1.joystickActivityClass, modulesCollector.autoPlayerClass2.joystickActivityClass
        });
    }

    @Override
    public void run() {
        while (running && !Thread.currentThread().isInterrupted())
        {

            try {
                Thread.sleep(35);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
    }
    public void showData(){
        //По нажатию кнопки меняем вывод
        int index = modulesCollector.semiAutoPlayerClass1.joystickActivityClass.tBPressed % modules.size();

        for (Module module : modules.get(index)){
            if(module.isInizialized) module.showData();
        };

        if (modulesCollector.teleOpModernized != null){
            modulesCollector.teleOpModernized.extShow();
        }
    }

}
