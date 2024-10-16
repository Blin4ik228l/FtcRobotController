package org.firstinspires.ftc.teamcode.OpModes.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OpModes.Robot;
import org.firstinspires.ftc.teamcode.RobotCore.RobotUtils.RobotAlliance;
import org.firstinspires.ftc.teamcode.RobotCore.RobotUtils.RobotCore;
import org.firstinspires.ftc.teamcode.RobotCore.RobotUtils.RobotMode;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.StdArgs;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.Task;
import org.firstinspires.ftc.teamcode.Utils.Position;


@TeleOp
public class TeleopTest extends OpMode {

    Robot robot = new Robot(RobotMode.TELEOP, RobotAlliance.BLUE, hardwareMap);

    @Override
    public void init() {
        robot.init();
        robot.odometry.setGlobalPosition(new Position(0,0,0));

        Task newtask = new Task(robot.example, new StdArgs(), 5, Task.taskStartMode.START_WITH_PREVIOUS);
        robot.taskManager.addTask(newtask);
    }

    @Override
    public void loop() {

    }
}
