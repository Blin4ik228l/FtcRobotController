package org.firstinspires.ftc.teamcode.Game.Robot.Modules.Unused;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Game.Robot.Modules.Module;

public class Metry implements Module {
    private final OpMode op;

    private volatile Telemetry telemetry;

    public Metry(OpMode op){
        this.op = op;
    }
    @Override
    public void init() {
        this.telemetry = op.telemetry;

        telemetry.addLine("Telemetry Inited");
    }

    public Telemetry getTelemetry() {
        return telemetry;
    }
}
