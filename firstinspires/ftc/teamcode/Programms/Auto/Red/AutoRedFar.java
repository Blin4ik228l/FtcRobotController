package org.firstinspires.ftc.teamcode.Programms.Auto.Red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Programms.Auto.LinearOpModeModernized;
import org.firstinspires.ftc.teamcode.MainParts.Examples.GeneralInformation;

@Autonomous(name = "RedFarWall", group = "Red", preselectTeleOp = "Red")
public class AutoRedFar extends LinearOpModeModernized {
    @Override
    public void initialization() {
        generalInformation = new GeneralInformation(GeneralInformation.ProgramName.Auto, GeneralInformation.Color.Red, GeneralInformation.StartPos.Far_from_wall);
    }

    @Override
    public void extRun() {

    }
}
