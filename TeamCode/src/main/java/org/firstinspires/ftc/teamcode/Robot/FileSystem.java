package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Modules.Examples.Players.Pl2.AutoPlayerClass;
import org.firstinspires.ftc.teamcode.Modules.Types.Module;
import org.firstinspires.ftc.teamcode.Modules.Types.UpdatableModule;
import java.nio.file.Files;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.StandardOpenOption;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

public class FileSystem extends UpdatableModule {
    public FileSystem(AutoPlayerClass autoPlayerClass, RobotClass robotClass, OpMode op){
        super(op.telemetry);

        this.autoPlayerClass = autoPlayerClass;
        this.robotClass = robotClass;

        file = AppUtil.getInstance().getSettingsFile("mydata.txt");
    }

    private File file;
    List<String> logBuffer = new ArrayList<>();
    private AutoPlayerClass autoPlayerClass;
    private RobotClass robotClass;
    private int attempts;

    @Override
    public void update() {

        String s1 = String.format("Target speed:%.2f Cur:%.2f After shot:%.2f Time:%.2f %n",autoPlayerClass.targetRadSpeedRed * 19.2, autoPlayerClass.collector.motors.curVel, autoPlayerClass.speeds[2], robotClass.innerRunTime.milliseconds());
//      String s2 = String.format("Target speed:%.2f Cur:%.2f After shot:%.2f Time:%.2f %n",autoPlayerClass.speeds[3],autoPlayerClass.speeds[4], autoPlayerClass.speeds[5], robotClass.innerRunTime.milliseconds());
//      String s3 = String.format("Target speed:%.2f Cur:%.2f After shot:%.2f Time:%.2f %n", autoPlayerClass.speeds[6],autoPlayerClass.speeds[7], autoPlayerClass.speeds[8], robotClass.innerRunTime.milliseconds());
        logBuffer.add(s1);
    }

    public void delete(){
        file.delete();
    }
    public void stop(){
        try (FileWriter fw = new FileWriter(file, true); ) {
            for (String line : logBuffer) fw.write(line);
        }catch (IOException e) {
            telemetry.addData("Ошибка", e.getMessage());
        }
    }
}
