package org.firstinspires.ftc.teamcode.Programms.Auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ModulesAndContainers.Containers.OpDataContainer;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.GeneralInformation;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Players.PL0.MainSystem;

public abstract class LinearOpModeModernized extends LinearOpMode {
    public GeneralInformation generalInformation;
    public MainSystem mainSystem;
    @Override
    public void runOpMode() throws InterruptedException {
        initialization();
        mainSystem = new MainSystem(generalInformation, this);
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
