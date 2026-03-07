package org.firstinspires.ftc.teamcode.Programms.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.ModulesAndContainers.Containers.OpDataContainer;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.GeneralInformation;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Players.PL0.MainSystem;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Config.MainFile;

public abstract class TeleOpModernized extends OpMode {
    public GeneralInformation generalInformation;
    public MainFile mainFile;
    public MainSystem mainSystem;

    public void initAfterRobot(){
        mainFile = new MainFile(this, generalInformation);
        mainSystem = new MainSystem(mainFile);
    }
    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        mainSystem.startExecuting();
//        telemetry.setAutoClear(false);
    }
    @Override
    public void loop() {
        mainSystem.execute();
        extShow();
        mainSystem.showData();
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

