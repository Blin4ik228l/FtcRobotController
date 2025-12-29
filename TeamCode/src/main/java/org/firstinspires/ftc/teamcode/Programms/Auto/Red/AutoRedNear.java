package org.firstinspires.ftc.teamcode.Programms.Auto.Red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Programms.Auto.LinearOpModeModernized;
import org.firstinspires.ftc.teamcode.Robot.RobotClass;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DriveTrain.DrivetrainParts.TeamClass;

@Autonomous(name = "RedNearWall", group = "Red", preselectTeleOp = "Red")
public class AutoRedNear extends LinearOpModeModernized {
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotClass(TeamClass.Color.Red, TeamClass.StartPos.Near_wall, this);

        run();
    }
}
