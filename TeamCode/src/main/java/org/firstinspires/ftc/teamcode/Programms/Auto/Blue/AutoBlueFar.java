package org.firstinspires.ftc.teamcode.Programms.Auto.Blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Programms.Auto.LinearOpModeModernized;
import org.firstinspires.ftc.teamcode.Robot.RobotClass;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DrivetrainParts.TeamColorClass;

@Autonomous(name = "BlueFarWall", group = "Blue", preselectTeleOp = "Blue")
public class AutoBlueFar extends LinearOpModeModernized {
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotClass(TeamColorClass.Color.Blue, TeamColorClass.StartPos.Far_from_wall, this);

        run();
    }
}
