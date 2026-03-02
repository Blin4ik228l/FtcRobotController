package org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Joysticks.Extenders;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Joysticks.JoystickActivityClass;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Config.MainFile;

public class Joystick2 extends JoystickActivityClass {
    public Joystick2(MainFile mainFile) {
        super(mainFile);
        playersGamepad = mainFile.op.gamepad2;
    }
}
