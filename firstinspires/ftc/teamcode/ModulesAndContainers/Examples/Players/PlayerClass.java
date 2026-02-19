package org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Players;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Joysticks.JoystickActivityClass;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Extenders.ExecutableModule;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.GeneralInformation;

public abstract class PlayerClass extends ExecutableModule {
    public GeneralInformation generalInformation;
    public JoystickActivityClass joystickActivityClass;
    public PlayerClass(GeneralInformation generalInformation, OpMode op) {
        super(op);
        this.generalInformation = generalInformation;
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
