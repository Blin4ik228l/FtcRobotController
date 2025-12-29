package org.firstinspires.ftc.teamcode.Programms.Auto.Blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Programms.Auto.LinearOpModeModernized;
import org.firstinspires.ftc.teamcode.Robot.RobotClass;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DriveTrain.DrivetrainParts.TeamClass;

@Autonomous(name = "BlueNearWall", group = "Blue", preselectTeleOp = "Blue")
public class AutoBlueNear extends LinearOpModeModernized {
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotClass(TeamClass.Color.Blue, TeamClass.StartPos.Near_wall, this);

        run();
    }
}
