package org.firstinspires.ftc.teamcode.Modules.Types;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Modules.Types.Interfaces.InterUpdate;

public class UpdatableModule extends Module implements InterUpdate {
    public UpdatableModule(OpMode op) {
        super(op);
    }
    @Override
    public void update() {

    }
}
