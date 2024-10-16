package org.firstinspires.ftc.teamcode.OpModes.Autonoms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.OpModes.Robot;
import org.firstinspires.ftc.teamcode.RobotCore.RobotUtils.RobotAlliance;
import org.firstinspires.ftc.teamcode.RobotCore.RobotUtils.RobotMode;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.StdArgs;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.Task;
import org.firstinspires.ftc.teamcode.Utils.Position;

@Autonomous(name = "RedLeft", group = "Red", preselectTeleOp = "TeleOpRed")
public class AutoRedLeft extends OpMode {
    Robot robot;

    /**
     *  Метод вызывается один раз при нажатии INIT
     */
    @Override
    public void init() {
        robot = new Robot(RobotMode.TELEOP, RobotAlliance.BLUE, this);

        robot.init();
        robot.odometry.setGlobalPosition(new Position(0,0,0));

        Task newtask = new Task(robot.example, new StdArgs(), 5, Task.taskStartMode.START_WITH_PREVIOUS);
        robot.taskManager.addTask(newtask);
    }

    /**
     *  Метод крутится в цикле, ожидая нажатия START
     */
    @Override
    public void init_loop() {
        super.init_loop();
    }

    /**
     *  Метод крутится в цикле после нажатия START
     */
    @Override
    public void loop() {

    }

    /**
     *  Метод вызывается один раз при нажатии STOP
     */
    @Override
    public void stop() {
        super.stop();
    }
}
