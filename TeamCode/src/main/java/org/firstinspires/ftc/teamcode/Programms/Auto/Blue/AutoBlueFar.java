package org.firstinspires.ftc.teamcode.Programms.Auto.Blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.MainParts.Examples.Players.Enums.TeamAliance;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.Odometry.Parts.MathUtils.Position2D;
import org.firstinspires.ftc.teamcode.Programms.Auto.LinearOpModeModernized;
import org.firstinspires.ftc.teamcode.MainParts.Examples.GeneralInformation;

@Autonomous(name = "BlueFarWall", group = "Blue", preselectTeleOp = "Blue")
public class AutoBlueFar extends LinearOpModeModernized {
    @Override
    public void initialization() {
        generalInformation = new GeneralInformation(GeneralInformation.ProgramName.Auto, TeamAliance.BLUE, GeneralInformation.StartPos.Far_from_wall);
        startPos = new Position2D(140, -15, 0);
    }

    @Override
    public void extRun() {

    }
}
