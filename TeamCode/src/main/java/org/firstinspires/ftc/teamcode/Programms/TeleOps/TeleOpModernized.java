package org.firstinspires.ftc.teamcode.Programms.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Modules.Types.ExecutableModule;
import org.firstinspires.ftc.teamcode.Modules.Types.UpdatableModule;

public abstract class TeleOpModernized extends OpMode {
    public UpdatableModule moduleJoystickActivityPlayer1, moduleRobot, moduleInnerWarden;
    public ExecutableModule modulePlayer1, moduleAutomatic;
    public void updateAll() {
        moduleJoystickActivityPlayer1.update();
        moduleRobot.update();
        moduleInnerWarden.update();
    }
    public void executeAll(){
        modulePlayer1.execute();
        moduleAutomatic.execute();
    }
    public void showAll(){
        moduleJoystickActivityPlayer1.showData();

        moduleRobot.showData();

        moduleAutomatic.showData();
    }
}

