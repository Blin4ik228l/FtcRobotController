package org.firstinspires.ftc.teamcode.Modules.Players;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Modules.Module;

public abstract class PlayerClass extends Module {
    public JoystickActivity joystickActivity;
    public PlayerClass(JoystickActivity joystickActivity, Telemetry telemetry) {
        super(telemetry);
        this.joystickActivity = joystickActivity;
    }
}
