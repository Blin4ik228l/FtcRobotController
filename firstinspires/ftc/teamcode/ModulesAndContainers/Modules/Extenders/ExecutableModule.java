package org.firstinspires.ftc.teamcode.MainParts.Modules.Extenders;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.MainParts.Modules.Module;

public abstract class ExecutableModule extends Module {
    public ExecutableModule(OpMode op) {
        super(op);
    }
    public abstract void execute();
}
