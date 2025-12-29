package org.firstinspires.ftc.teamcode.Programms.TeleOps.Blue;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Programms.TeleOps.TeleOpModernized;
import org.firstinspires.ftc.teamcode.Robot.RobotClass;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DriveTrain.DrivetrainParts.TeamClass;

@TeleOp(name = "Blue", group = "Blue")
public class TeleOpBlue extends TeleOpModernized {
    @Override
    public void init() {
        robot = new RobotClass(TeamClass.Color.Blue, TeamClass.StartPos.Nevermind, this);

        initAfterRobot();
    }
}
