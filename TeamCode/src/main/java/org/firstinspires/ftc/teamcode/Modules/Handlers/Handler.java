package org.firstinspires.ftc.teamcode.Modules.Handlers;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Modules.Module;

public abstract class Handler extends Module {
    public Handler(Telemetry telemetry){
        super(telemetry);
    }

    public boolean isDone = false;
    public void execute(){

    }

    public void showData(){

    }
}
