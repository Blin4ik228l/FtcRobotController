package org.firstinspires.ftc.teamcode.Modules;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Consts;
import org.firstinspires.ftc.teamcode.ConstsTeleskope;
import org.firstinspires.ftc.teamcode.Programms.TeleOps.TeleOp;

public abstract class Module implements ConstsTeleskope, Consts {
    public Module(Telemetry telemetry){
        this.telemetry = telemetry;
    }
    public Telemetry telemetry;
    public void showData(){};

}
