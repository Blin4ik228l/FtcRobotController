package org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Joysticks.Extenders;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Joysticks.JoystickActivityClass;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Config.MainFile;

public class Joystick1 extends JoystickActivityClass {
    public Joystick1(MainFile mainFile) {
        super(mainFile);
        playersGamepad = mainFile.op.gamepad1;
    }
}
