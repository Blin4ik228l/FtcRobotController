package org.firstinspires.ftc.teamcode.Modules.Types;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class UpdatableModule extends Module implements InterUpdate{
    public UpdatableModule(OpMode op) {
        super(op);
    }
    @Override
    public void update() {

    }
}
