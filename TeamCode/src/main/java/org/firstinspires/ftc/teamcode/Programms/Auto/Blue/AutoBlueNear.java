package org.firstinspires.ftc.teamcode.Programms.Auto.Blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.MainParts.Examples.Players.Enums.TeamAliance;
import org.firstinspires.ftc.teamcode.Programms.Auto.LinearOpModeModernized;
import org.firstinspires.ftc.teamcode.MainParts.Examples.GeneralInformation;

@Autonomous(name = "BlueNearWall", group = "Blue", preselectTeleOp = "Blue")
public class AutoBlueNear extends LinearOpModeModernized {
    @Override
    public void initialization() {
        generalInformation = new GeneralInformation(GeneralInformation.ProgramName.Auto, TeamAliance.BLUE, GeneralInformation.StartPos.Near_wall);
    }

    @Override
    public void extRun() {

    }
}
