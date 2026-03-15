package org.firstinspires.ftc.teamcode.MainParts.Examples.Joysticks.Extenders;

import org.firstinspires.ftc.teamcode.MainParts.Examples.Joysticks.JoystickActivityClass;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.Config.MainFile;

public class Joystick2 extends JoystickActivityClass {

    public Joystick2(boolean isThisExecutingOtherModules) {
        super(isThisExecutingOtherModules);
        playersGamepad = MainFile.op.gamepad2;
    }
}
