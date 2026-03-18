package org.firstinspires.ftc.teamcode.Programms.Auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MainParts.Examples.GeneralInformation;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Players.Enums.ProgramStage;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Players.PL0.MainSystem;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.Config.MainFile;

import java.util.concurrent.CyclicBarrier;

public abstract class LinearOpModeModernized extends LinearOpMode {
    public GeneralInformation generalInformation;
    public MainSystem mainSystem;
    public MainFile mainFile;
    private CyclicBarrier barrier = new CyclicBarrier(3);
    @Override
    public void runOpMode() throws InterruptedException {
        initialization();
        mainFile = new MainFile(this, generalInformation);
        mainSystem = new MainSystem(barrier);
        run();
    }
    public void run(){
        generalInformation.programStage = ProgramStage.Init_loop;
        while (!isStarted() && !isStopRequested()){
            mainSystem.showData();
            telemetry.update();
        }

        mainSystem.startExecuting();
        generalInformation.programStage = ProgramStage.Main_loop;
        while (!isStopRequested() && opModeIsActive()){
            extRun();
            mainSystem.execute();
            mainSystem.showData();
            telemetry.update();
        }
        mainSystem.interrupt();
    }
    //Ваша будущая реализация
    public abstract void initialization();
    //ваша будущая реализация
    public abstract void extRun();
}
