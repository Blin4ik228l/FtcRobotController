package org.firstinspires.ftc.teamcode.Modules.Examples.Players;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Modules.Types.ExecutableModule;

public abstract class PlayerClass extends ExecutableModule {
    public JoystickActivityClass joystickActivityClass;
    public PlayerClass(JoystickActivityClass joystickActivityClass, Telemetry telemetry) {
        super(telemetry);
        this.joystickActivityClass = joystickActivityClass;
    }
}
