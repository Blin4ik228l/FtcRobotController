package org.firstinspires.ftc.teamcode.Programms.TeleOps.Blue;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Programms.TeleOps.TeleOpModernized;
import org.firstinspires.ftc.teamcode.Robot.RobotClass;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DrivetrainParts.TeamColorClass;

@TeleOp(name = "Blue", group = "Blue")
public class TeleOpBlue extends TeleOpModernized {
    @Override
    public void init() {
        robot = new RobotClass(TeamColorClass.Color.Blue, TeamColorClass.StartPos.Nevermind, this);

        initAfterRobot();
    }
}
