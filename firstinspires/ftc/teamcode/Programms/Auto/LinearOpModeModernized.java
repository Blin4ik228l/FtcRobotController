package org.firstinspires.ftc.teamcode.Programms.Auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MainParts.Containers.OpDataContainer;
import org.firstinspires.ftc.teamcode.MainParts.Examples.GeneralInformation;

public abstract class LinearOpModeModernized extends LinearOpMode {
    public GeneralInformation generalInformation;
    public OpDataContainer opDataContainer;
    @Override
    public void runOpMode() throws InterruptedException {
        initialization();
        opDataContainer = new OpDataContainer(generalInformation, this);
        waitForStart();
        run();
    }
    public void run(){
        while (!isStarted() && !isStopRequested()){
           opDataContainer.telemetryContainer.showData();
           telemetry.update();
        }
        opDataContainer.resetTimer();
        opDataContainer.start();

        while (!isStopRequested() && opModeIsActive()){
            extRun();
            opDataContainer.telemetryContainer.showData();
            telemetry.update();
        }
    }
    //Ваша будущая реализация
    public abstract void initialization();
    //ваша будущая реализация
    public abstract void extRun();
}
