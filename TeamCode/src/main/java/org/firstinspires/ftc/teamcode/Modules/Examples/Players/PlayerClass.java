package org.firstinspires.ftc.teamcode.Modules.Examples.Players;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Modules.Joysticks.JoystickActivityClass;
import org.firstinspires.ftc.teamcode.Modules.Types.ExecutableModule;

public abstract class PlayerClass extends ExecutableModule {
    public JoystickActivityClass joystickActivityClass;
    public PlayerClass(JoystickActivityClass joystickActivityClass, OpMode op) {
        super(op);
        this.joystickActivityClass = joystickActivityClass;
    }
}
