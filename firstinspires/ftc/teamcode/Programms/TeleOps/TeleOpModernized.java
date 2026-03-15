package org.firstinspires.ftc.teamcode.Programms.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.MainParts.Containers.OpDataContainer;
import org.firstinspires.ftc.teamcode.MainParts.Examples.GeneralInformation;

public abstract class TeleOpModernized extends OpMode {
    public GeneralInformation generalInformation;
    public OpDataContainer opDataContainer;

    public void initAfterRobot(){
        opDataContainer = new OpDataContainer(generalInformation, this);
    }
    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        opDataContainer.start();
        opDataContainer.resetTimer();
    }

    @Override
    public void loop() {
        opDataContainer.telemetryContainer.showData();
        extShow();
    }

    @Override
    public void stop() {
        opDataContainer.interrupt();
    }

    //Ваша будущая реализация
    public abstract void extUpdate();
    public abstract void extExecute();
    public abstract void extShow();
}

