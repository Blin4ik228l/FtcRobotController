package org.firstinspires.ftc.teamcode.Programms.Auto.Red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.MainParts.Examples.Players.Enums.TeamAliance;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.Odometry.Parts.MathUtils.Position2D;
import org.firstinspires.ftc.teamcode.Programms.Auto.LinearOpModeModernized;
import org.firstinspires.ftc.teamcode.MainParts.Examples.GeneralInformation;

@Autonomous(name = "RedNearWall", group = "Red", preselectTeleOp = "Red")
public class AutoRedNear extends LinearOpModeModernized {
    @Override
    public void initialization() {
        generalInformation = new GeneralInformation(GeneralInformation.ProgramName.Auto, TeamAliance.RED, GeneralInformation.StartPos.Near_wall);
        startPos = new Position2D(-140, 70, 0);
    }

    @Override
    public void extRun() {

    }
}
