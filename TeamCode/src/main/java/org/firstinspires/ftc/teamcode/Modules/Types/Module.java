package org.firstinspires.ftc.teamcode.Modules.Types;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ConstansOrMagicNumbers.AnotherConsts;
import org.firstinspires.ftc.teamcode.ConstansOrMagicNumbers.Delays;
import org.firstinspires.ftc.teamcode.ConstansOrMagicNumbers.kPIDS;
import org.firstinspires.ftc.teamcode.ConstansOrMagicNumbers.ServoPositions;
import org.firstinspires.ftc.teamcode.Robot.GeneralInformation;

public abstract class Module  implements ServoPositions, AnotherConsts, Delays, kPIDS {
    public Module(Telemetry telemetry){
        this.telemetry = telemetry;
    }
    public Telemetry telemetry;
    public void showData(){};

}
