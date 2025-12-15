package org.firstinspires.ftc.teamcode.Programms.TeleOps.Red;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Programms.TeleOps.TeleOpModernized;
import org.firstinspires.ftc.teamcode.Robot.RobotClass;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DrivetrainParts.TeamColorClass;

@TeleOp(name = "Red", group = "Red")
public class TeleOpRed extends TeleOpModernized {
    @Override
    public void init() {
        robot = new RobotClass(TeamColorClass.Color.Red, TeamColorClass.StartPos.Nevermind, this);

        initAfterRobot();
    }
}
