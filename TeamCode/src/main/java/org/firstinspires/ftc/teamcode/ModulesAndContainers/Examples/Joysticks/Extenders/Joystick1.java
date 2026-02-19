package org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Joysticks.Extenders;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Joysticks.JoystickActivityClass;

public class Joystick1 extends JoystickActivityClass {
    public Joystick1(OpMode op) {
        super(op);
        playersGamepad = op.gamepad1;
    }
}
