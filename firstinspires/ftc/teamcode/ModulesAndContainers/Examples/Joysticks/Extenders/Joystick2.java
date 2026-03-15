package org.firstinspires.ftc.teamcode.MainParts.Examples.Joysticks.Extenders;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.MainParts.Examples.Joysticks.JoystickActivityClass;

public class Joystick2 extends JoystickActivityClass {
    public Joystick2(OpMode op) {
        super(op);
        playersGamepad = op.gamepad2;
    }
}
