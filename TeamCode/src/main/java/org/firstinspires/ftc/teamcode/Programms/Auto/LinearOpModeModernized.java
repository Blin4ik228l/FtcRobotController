package org.firstinspires.ftc.teamcode.Programms.Auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ModulesAndContainers.Containers.OpDataContainer;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.GeneralInformation;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Players.PL0.MainSystem;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Config.MainFile;

public abstract class LinearOpModeModernized extends LinearOpMode {
    public GeneralInformation generalInformation;
    public MainSystem mainSystem;
    public MainFile mainFile;
    @Override
    public void runOpMode() throws InterruptedException {
        initialization();

        mainFile = new MainFile(this, generalInformation);
        mainSystem = new MainSystem(mainFile);
        waitForStart();
        run();
    }
    public void run(){
        while (!isStarted() && !isStopRequested()){
            mainSystem.showData();
            telemetry.update();
        }

        mainSystem.startExecuting();

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
