package org.firstinspires.ftc.teamcode.MainParts.Examples.Players;

import org.firstinspires.ftc.teamcode.MainParts.Examples.Joysticks.JoystickActivityClass;
import org.firstinspires.ftc.teamcode.MainParts.Examples.GeneralInformation;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.Config.MainFile;
import org.firstinspires.ftc.teamcode.MainParts.Modules.ExecutorModule;

public abstract class PlayerClass extends ExecutorModule {
    protected GeneralInformation generalInformation;
    protected JoystickActivityClass joystickActivityClass;
    public PlayerClass() {
        this.generalInformation = MainFile.generalInformation;
    }
    public void setJoystickActivityClass(JoystickActivityClass joystickActivityClass) {
        this.joystickActivityClass = joystickActivityClass;
    }
    public void checkButtons(){
        if(joystickActivityClass.buttonA){
            buttonAReleased();
        }else {
            buttonAUnReleased();
        }

        if(joystickActivityClass.buttonB){
            buttonBReleased();
        }else {
            buttonBUnReleased();
        }

        if(joystickActivityClass.buttonX){
            buttonXReleased();
        }else {
            buttonXUnReleased();
        }

        if(joystickActivityClass.buttonY){
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
