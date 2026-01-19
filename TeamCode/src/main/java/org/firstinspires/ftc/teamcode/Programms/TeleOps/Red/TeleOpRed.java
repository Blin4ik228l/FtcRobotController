package org.firstinspires.ftc.teamcode.Programms.TeleOps.Red;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Programms.TeleOps.TeleOpModernized;
import org.firstinspires.ftc.teamcode.Robot.GeneralInformation;
import org.firstinspires.ftc.teamcode.Robot.RobotClass;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DriveTrain.DrivetrainParts.TeamClass;

@TeleOp(name = "Red", group = "Red")
public class TeleOpRed extends TeleOpModernized {
    @Override
    public void init() {
        robot = new RobotClass(this);
        robot.setGeneralInformation(GeneralInformation.ProgramName.TeleOp, GeneralInformation.Color.Red, GeneralInformation.StartPos.Nevermind);
        initAfterRobot();
    }
}
