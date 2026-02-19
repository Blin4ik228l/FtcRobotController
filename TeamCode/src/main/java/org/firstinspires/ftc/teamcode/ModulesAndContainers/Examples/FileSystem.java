package org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Containers.OpDataContainer;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Players.Pl2.AutoPlayerClass2;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;
import java.util.Locale;

public class FileSystem{
    //Данному классу требуется доступ ко всем объектам программы
    private File file;
    private List<String> logBuffer ;
    private Date date;
    public OpDataContainer opDataContainer;
    public FileSystem(OpDataContainer opDataContainer){
        this.opDataContainer = opDataContainer;

        file = AppUtil.getInstance().getSettingsFile("Data.txt");
        logBuffer = new ArrayList<>();
        date = new Date();
    }

    public void write() {
        AutoPlayerClass2 autoPlayerClass2 = opDataContainer.autoPlayerClass2;

//        String s1 = String.format("Target speed:%.2f Cur:%.2f After shot:%.2f Time:%.2f P:%s I:%s D:%s F:%s Global:%s Angle:%s Distance:%s %n",
//                autoPlayerClass.targetRadSpeedRed * 19.2, autoPlayerClass.collector.motors.curVel, autoPlayerClass.speeds[2], robotClass.innerRunTime.milliseconds(),
//                autoPlayerClass.collector.motors.P, autoPlayerClass.collector.motors.I, autoPlayerClass.collector.motors.D, autoPlayerClass.collector.motors.F,
//                date.getTime(), Math.toDegrees(autoPlayerClass.targetAngle), autoPlayerClass.range);

        String targSpeed = String.valueOf(autoPlayerClass2.targetRadSpeed * 19.2);
        String curVel = String.valueOf(autoPlayerClass2.hoodedShoter.motors.curVel);
        String kPIDF = String.format("P: %s I: %s D: %s F: %s", autoPlayerClass2.hoodedShoter.motors.getPIDF()[0], autoPlayerClass2.hoodedShoter.motors.getPIDF()[1], autoPlayerClass2.hoodedShoter.motors.getPIDF()[2], autoPlayerClass2.hoodedShoter.motors.getPIDF()[3]);
        String params = String.format(Locale.ENGLISH, "Angle: %.2f Range: %.2f", Math.toDegrees(autoPlayerClass2.targetAngle), autoPlayerClass2.range);
        String states = String.format("General: %s Another: %s", autoPlayerClass2.generalState.toString(), autoPlayerClass2.anotherStates.toString());
        String dat = String.valueOf(date.getTime());

        logBuffer.add(String.format("Speeds[T: %s C: %s]", targSpeed, curVel));
        logBuffer.add(kPIDF);
        logBuffer.add(params);
        logBuffer.add(states);
        logBuffer.add(String.format("Date in milliseconds: %s", dat));
        logBuffer.add("");
    }

    public void delete(){
        file.delete();
    }
    public void stop(){
        try (FileWriter fw = new FileWriter(file, true); ) {
            for (String line : logBuffer) fw.write(line);
        }catch (IOException e) {
        }
    }
}
