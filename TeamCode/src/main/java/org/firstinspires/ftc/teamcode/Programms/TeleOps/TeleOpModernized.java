package org.firstinspires.ftc.teamcode.Programms.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MainParts.Examples.GeneralInformation;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Players.Enums.ProgramStage;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Players.PL0.MainSystem;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.Config.MainFile;

public abstract class TeleOpModernized extends OpMode {
    public GeneralInformation generalInformation;
    public MainFile mainFile;
    public MainSystem mainSystem;

    public void initAfterRobot(){
        mainFile = new MainFile(this, generalInformation);
        mainSystem = new MainSystem();
        generalInformation.programStage = ProgramStage.Init_loop;
    }
    @Override
    public void init_loop() {
        mainSystem.execute();
        mainSystem.showData();
    }

    @Override
    public void start() {
        mainSystem.startExecuting();
//        telemetry.setAutoClear(false);
        generalInformation.programStage = ProgramStage.Main_loop;
    }
    int targetFrequencyHz = 60;
    long targetPeriodNs = 1_000_000_000 / targetFrequencyHz;
    long targetSleepMs = targetPeriodNs / 1_000_000;
    @Override
    public void loop() {
        long start = System.nanoTime();

        mainSystem.execute();
        extShow();
        mainSystem.showData();

        long elapsed = System.nanoTime() - start;
        long sleepTime = targetPeriodNs - elapsed;

        if (sleepTime > 0){
            try {
                Thread.sleep(targetSleepMs);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
    }

    @Override
    public void stop() {
        mainSystem.interrupt();
    }

    //Ваша будущая реализация
    public abstract void extUpdate();
    public abstract void extExecute();
    public abstract void extShow();
}

