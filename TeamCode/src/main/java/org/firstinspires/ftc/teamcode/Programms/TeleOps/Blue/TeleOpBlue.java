package org.firstinspires.ftc.teamcode.Programms.TeleOps.Blue;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Programms.TeleOps.TeleOpModernized;
import org.firstinspires.ftc.teamcode.Robot.GeneralInformation;
import org.firstinspires.ftc.teamcode.Robot.RobotClass;

@TeleOp(name = "Blue", group = "Blue")
public class TeleOpBlue extends TeleOpModernized {
    @Override
    public void init() {
        new GeneralInformation(GeneralInformation.ProgramName.TeleOp, GeneralInformation.Color.Blue, GeneralInformation.StartPos.Nevermind);
        robot = new RobotClass(this);

        initAfterRobot();
    }
}
