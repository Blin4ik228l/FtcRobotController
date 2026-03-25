package org.firstinspires.ftc.teamcode.Programms.Auto.Red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.MainParts.Examples.Players.Enums.TeamAliance;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.Odometry.Parts.MathUtils.Position2D;
import org.firstinspires.ftc.teamcode.Programms.Auto.LinearOpModeModernized;
import org.firstinspires.ftc.teamcode.MainParts.Examples.GeneralInformation;

@Autonomous(name = "RedFarWall", group = "Red", preselectTeleOp = "Red")
public class AutoRedFar extends LinearOpModeModernized {
    @Override
    public void initialization() {
        generalInformation = new GeneralInformation(GeneralInformation.ProgramName.Auto, TeamAliance.RED, GeneralInformation.StartPos.Far_from_wall);
        startPos = new Position2D(140, 15, 0);
    }

    @Override
    public void extRun() {

    }
}
