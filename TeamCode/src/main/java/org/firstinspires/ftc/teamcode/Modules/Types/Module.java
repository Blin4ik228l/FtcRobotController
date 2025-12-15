package org.firstinspires.ftc.teamcode.Modules.Types;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ConstansOrMagicNumbers.AnotherConsts;
import org.firstinspires.ftc.teamcode.ConstansOrMagicNumbers.PositionConsts;

public abstract class Module implements PositionConsts, AnotherConsts {
    public Module(Telemetry telemetry){
        this.telemetry = telemetry;
    }
    public Telemetry telemetry;
    public void showData(){};

}
