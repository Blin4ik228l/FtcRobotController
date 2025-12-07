package org.firstinspires.ftc.teamcode.Modules.Examples.Players;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Modules.Types.ExecutableModule;

public abstract class PlayerClass extends ExecutableModule {
    public JoystickActivity joystickActivity;
    public PlayerClass(JoystickActivity joystickActivity, Telemetry telemetry) {
        super(telemetry);
        this.joystickActivity = joystickActivity;
    }
}
