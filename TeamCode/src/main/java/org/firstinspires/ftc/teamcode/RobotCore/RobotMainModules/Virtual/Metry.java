package org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules.Virtual;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.robocol.TelemetryMessage;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules.Module;

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
