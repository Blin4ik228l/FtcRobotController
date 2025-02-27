package org.firstinspires.ftc.teamcode.OpModes.Autonoms.Classes.Blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Consts.Consts;
import org.firstinspires.ftc.teamcode.Consts.ConstsTeleskope;
import org.firstinspires.ftc.teamcode.Consts.RewardsForActions;
import org.firstinspires.ftc.teamcode.OpModes.Robot;
import org.firstinspires.ftc.teamcode.RobotCore.RobotUtils.RobotAlliance;
import org.firstinspires.ftc.teamcode.RobotCore.RobotUtils.RobotMode;
import org.firstinspires.ftc.teamcode.RobotCore.Utils.Position;

@Autonomous(name = "BlueLeft", group = "Blue", preselectTeleOp = "BlueMeow")
public class AutoBlueLeft extends LinearOpMode implements ConstsTeleskope, Consts, RewardsForActions {
    Robot robot;
//    Position pos1 = new Position(70, -83 , 0);
//    Position pos2 = new Position( 78, -83, 0);
//    Position pos3 = new Position( 65, -80, 90);
//    Position pos4 = new Position( 75, 15,45);

    Position pos1 = new Position(30, 40,135);//25, 45, 135
    Position pos2 = new Position(30, 32,0);//25, 33, 0
    Position pos3 = new Position(30, 63, 0);//25, 60, 0
    Position pos4 = new Position(30, 40,-225);//25, 60, -225
    Position pos5 = new Position(150, 0, 0); //25, -240, 90 МОЖЕТ БЫТЬ ОТСТУП 5 СМ ОТ КАРЗИНЫ
    Position pos6 = new Position(150, -40, 0);
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(RobotMode.AUTO,RobotAlliance.BLUE, this, new Position(0,0,0));
        robot.init();

//

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            robot.taskManager.forAuto();
        }

    }
}
