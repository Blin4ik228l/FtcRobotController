package org.firstinspires.ftc.teamcode.OpModes.Autonoms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.OpModes.Robot;
import org.firstinspires.ftc.teamcode.RobotCore.RobotUtils.RobotAlliance;
import org.firstinspires.ftc.teamcode.RobotCore.RobotUtils.RobotMode;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.StandartArgs;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.Task;
import org.firstinspires.ftc.teamcode.RobotCore.Utils.Position;

@Autonomous(name = "RightBlue", group = "Blue", preselectTeleOp = "TeleOpBlue")
public class AutoBlueRight extends LinearOpMode {
    Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {

    }

    @Override
    public void waitForStart() {
        super.waitForStart();
        robot = new Robot(RobotMode.TELEOP, RobotAlliance.BLUE, this);

        robot.init();
        robot.odometry.setGlobalPosition(new Position(0,0,0));

        Task newtask = new Task(robot.example, new StandartArgs(), 5, Task.taskStartMode.START_WITH_PREVIOUS);
        robot.taskManager.addTask(newtask);
    }

}
