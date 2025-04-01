package org.firstinspires.ftc.teamcode.Game.Players.MultiPlayer.Driver1;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Game.Players.Joystick.ClassJoystick;

public class Joystick1 extends ClassJoystick {
    public Joystick1(Gamepad gamepad) {
        super(gamepad);
    }

    @Override
    public void run() {
        while (!isInterrupted()){
            checkButtonsActivity();}
    }
}
