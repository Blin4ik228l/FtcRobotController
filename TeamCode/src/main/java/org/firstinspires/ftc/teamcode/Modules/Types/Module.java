package org.firstinspires.ftc.teamcode.Modules.Types;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ConstansOrMagicNumbers.Consts;
import org.firstinspires.ftc.teamcode.ConstansOrMagicNumbers.ConstsTeleskope;

public abstract class Module implements ConstsTeleskope, Consts {
    public Module(Telemetry telemetry){
        this.telemetry = telemetry;
    }
    public Telemetry telemetry;
    public void showData(){};

}
