package org.firstinspires.ftc.teamcode.Programms.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Modules.ExecutableModule;
import org.firstinspires.ftc.teamcode.Modules.MainModule;
import org.firstinspires.ftc.teamcode.Modules.UpdatableModule;

public abstract class TeleOpModernized extends OpMode {
    public UpdatableModule moduleJoystickActivityPlayer1, moduleRobot;
    public ExecutableModule modulePlayer1, moduleAutomatic, moduleInnerWarden;
    public void updateAll() {
        moduleJoystickActivityPlayer1.update();
        moduleRobot.update();
    }
    public void executeAll(){
        modulePlayer1.execute();
        moduleInnerWarden.execute();
        moduleAutomatic.execute();
    }
    public void showAll(){
        moduleJoystickActivityPlayer1.showData();

        moduleRobot.showData();

        moduleAutomatic.showData();
    }
}

