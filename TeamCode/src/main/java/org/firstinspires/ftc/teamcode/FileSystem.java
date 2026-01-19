package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Modules.Examples.Players.Pl2.AutoPlayerClass;
import org.firstinspires.ftc.teamcode.Modules.Types.UpdatableModule;
import org.firstinspires.ftc.teamcode.Robot.RobotClass;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;
import java.util.Locale;

public class FileSystem extends UpdatableModule {
    private File file;
    private List<String> logBuffer ;
    private AutoPlayerClass autoPlayerClass;
    private RobotClass robotClass;
    private Date date;
    public FileSystem(AutoPlayerClass autoPlayerClass, RobotClass robotClass, OpMode op){
        super(op.telemetry);

        this.autoPlayerClass = autoPlayerClass;
        this.robotClass = robotClass;

        file = AppUtil.getInstance().getSettingsFile("mydata.txt");
        logBuffer = new ArrayList<>();
        date = new Date();
    }


    @Override
    public void update() {
//        String s1 = String.format("Target speed:%.2f Cur:%.2f After shot:%.2f Time:%.2f P:%s I:%s D:%s F:%s Global:%s Angle:%s Distance:%s %n",
//                autoPlayerClass.targetRadSpeedRed * 19.2, autoPlayerClass.collector.motors.curVel, autoPlayerClass.speeds[2], robotClass.innerRunTime.milliseconds(),
//                autoPlayerClass.collector.motors.P, autoPlayerClass.collector.motors.I, autoPlayerClass.collector.motors.D, autoPlayerClass.collector.motors.F,
//                date.getTime(), Math.toDegrees(autoPlayerClass.targetAngle), autoPlayerClass.range);

        String targSpeed = String.valueOf(autoPlayerClass.targetRadSpeedRed * 19.2);
        String curVel = String.valueOf(autoPlayerClass.collector.motors.curVel);
        String velAfterShot = String.valueOf(autoPlayerClass.speeds[2]);
        String kPIDF = String.format("P: %s I: %s D: %s F: %s", autoPlayerClass.collector.motors.getPIDF()[0], autoPlayerClass.collector.motors.getPIDF()[1], autoPlayerClass.collector.motors.getPIDF()[2], autoPlayerClass.collector.motors.getPIDF()[3]);
        String params = String.format(Locale.ENGLISH, "Angle: %.2f Range: %.2f", Math.toDegrees(autoPlayerClass.targetAngle), autoPlayerClass.range);
        String states = String.format("General: %s Another: %s", autoPlayerClass.generalState.toString(), autoPlayerClass.anotherStates.toString());
        String dat = String.valueOf(date.getTime());

        logBuffer.add(String.format("Speeds[T: %s C: %s AS: %s]", targSpeed, curVel, velAfterShot));
        logBuffer.add(kPIDF);
        logBuffer.add(params);
        logBuffer.add(String.format("Date in milliseconds: %s", dat));
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
