package org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Players;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Joysticks.JoystickActivityClass;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.GeneralInformation;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Config.MainFile;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.ExecutorModule;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Extenders.ExecutingModule;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Module;

public abstract class PlayerClass extends ExecutorModule {
    protected GeneralInformation generalInformation;
    protected JoystickActivityClass joystickActivityClass;
    public PlayerClass(MainFile mainFile) {
        super(mainFile);
        this.generalInformation = mainFile.generalInformation;
    }
    public void setJoystickActivityClass(JoystickActivityClass joystickActivityClass) {
        this.joystickActivityClass = joystickActivityClass;
    }
    public void checkButtons(){
        if(joystickActivityClass.playersGamepad.a){
            buttonAReleased();
        }else {
            buttonAUnReleased();
        }

        if(joystickActivityClass.playersGamepad.b){
            buttonBReleased();
        }else {
            buttonBUnReleased();
        }

        if(joystickActivityClass.playersGamepad.x){
            buttonXReleased();
        }else {
            buttonXUnReleased();
        }

        if(joystickActivityClass.playersGamepad.y){
            buttonYReleased();
        }else {
            buttonYUnReleased();
        }
    }

    public abstract void buttonAReleased();
    public abstract void buttonAUnReleased();
    public abstract void buttonBReleased();
    public abstract void buttonBUnReleased();
    public abstract void buttonXReleased();
    public abstract void buttonXUnReleased();
    public abstract void buttonYReleased();
    public abstract void buttonYUnReleased();
}
