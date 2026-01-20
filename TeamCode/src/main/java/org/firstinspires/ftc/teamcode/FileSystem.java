package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Modules.Examples.Players.Pl2.AutoPlayerClass2;
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
    private AutoPlayerClass2 autoPlayerClass2;
    private RobotClass robotClass;
    private Date date;
    public FileSystem(AutoPlayerClass2 autoPlayerClass2, RobotClass robotClass, OpMode op){
        super(op);

        this.autoPlayerClass2 = autoPlayerClass2;
        this.robotClass = robotClass;

        file = AppUtil.getInstance().getSettingsFile("mydata.txt");
        logBuffer = new ArrayList<>();
        date = new Date();
    }

    @Override
    public void update() {
        String targSpeed = String.valueOf(autoPlayerClass2.targetRadSpeedRed * 19.2);
        String curVel = String.valueOf(autoPlayerClass2.collector.motors.curVel);
        String kPIDF = String.format("P: %s I: %s D: %s F: %s", autoPlayerClass2.collector.motors.getPIDF()[0], autoPlayerClass2.collector.motors.getPIDF()[1], autoPlayerClass2.collector.motors.getPIDF()[2], autoPlayerClass2.collector.motors.getPIDF()[3]);
        String params = String.format(Locale.ENGLISH, "Angle: %.2f Range: %.2f", Math.toDegrees(autoPlayerClass2.targetAngle), autoPlayerClass2.range);
        String states = String.format("General: %s Another: %s", autoPlayerClass2.generalState.toString(), autoPlayerClass2.anotherStates.toString());
        String dat = String.valueOf(robotClass.innerRunTime.seconds());

        logBuffer.add(String.format("Speeds[T: %s C: %s] %n", targSpeed, curVel));
        logBuffer.add(String.format("%s %n", kPIDF));
        logBuffer.add(String.format("%s %n", params));
        logBuffer.add(String.format("%s %n", states));
        logBuffer.add(String.format("Inner time: %s %n", dat));
        logBuffer.add(String.format("%n"));
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
