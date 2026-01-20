package org.firstinspires.ftc.teamcode.Programms.Auto.Blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Programms.Auto.LinearOpModeModernized;
import org.firstinspires.ftc.teamcode.Robot.GeneralInformation;
import org.firstinspires.ftc.teamcode.Robot.RobotClass;

@Autonomous(name = "BlueNearWall", group = "Blue", preselectTeleOp = "Blue")
public class AutoBlueNear extends LinearOpModeModernized {
    @Override
    public void runOpMode() throws InterruptedException {
        new GeneralInformation(GeneralInformation.ProgramName.Auto, GeneralInformation.Color.Blue, GeneralInformation.StartPos.Near_wall);

        robot = new RobotClass(this);
        run();
    }
}
