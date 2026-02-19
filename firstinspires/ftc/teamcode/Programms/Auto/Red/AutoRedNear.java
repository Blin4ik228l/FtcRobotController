package org.firstinspires.ftc.teamcode.Programms.Auto.Red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Programms.Auto.LinearOpModeModernized;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.GeneralInformation;

@Autonomous(name = "RedNearWall", group = "Red", preselectTeleOp = "Red")
public class AutoRedNear extends LinearOpModeModernized {
    @Override
    public void initialization() {
        generalInformation = new GeneralInformation(GeneralInformation.ProgramName.Auto, GeneralInformation.Color.Red, GeneralInformation.StartPos.Near_wall);
    }

    @Override
    public void extRun() {

    }
}
