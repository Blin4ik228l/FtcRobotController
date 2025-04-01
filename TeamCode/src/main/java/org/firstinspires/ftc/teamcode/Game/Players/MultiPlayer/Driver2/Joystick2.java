package org.firstinspires.ftc.teamcode.Game.Players.MultiPlayer.Driver2;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Game.Players.Joystick.ClassJoystick;

public class Joystick2 extends ClassJoystick {
    public Joystick2(Gamepad gamepad) {
        super(gamepad);
    }

    @Override
    public void run() {
        while (!isInterrupted()){
            checkButtonsActivity();}
    }

    public int dPadUpDownAction(){
        if(tDpadDownPressed == 1){
            tDpadUpPressed = 0;
            tDpadDownPressed = 0;
        }

        return tDpadUpPressed;
    }

    public int triggerLeftRightAction(){
        if(tLeftTriggerPressed == 1){
            tRightTriggerPressed = 0;
            tLeftTriggerPressed = 0;
        }
        return tLeftTriggerPressed;
    }
}
