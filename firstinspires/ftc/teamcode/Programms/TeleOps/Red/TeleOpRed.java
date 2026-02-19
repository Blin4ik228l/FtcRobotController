package org.firstinspires.ftc.teamcode.Programms.TeleOps.Red;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Programms.TeleOps.TeleOpModernized;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.GeneralInformation;

@TeleOp(name = "Red", group = "Red")
public class TeleOpRed extends TeleOpModernized {
    @Override
    public void init() {
        generalInformation = new GeneralInformation(GeneralInformation.ProgramName.TeleOp, GeneralInformation.Color.Red, GeneralInformation.StartPos.Nevermind);
        initAfterRobot();
    }

    @Override
    public void extUpdate() {

    }

    @Override
    public void extExecute() {

    }

    @Override
    public void extShow() {

    }
}
