package org.firstinspires.ftc.teamcode.OpModes.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math3.analysis.function.Abs;
import org.firstinspires.ftc.teamcode.Robot.TaskHandler.Args;
import org.firstinspires.ftc.teamcode.Robot.TaskHandler.Tasks;
import org.firstinspires.ftc.teamcode.Utils.CONSTS;
import org.firstinspires.ftc.teamcode.Utils.Position;
import org.firstinspires.ftc.teamcode.Robot.ROBOT;


@TeleOp
public class TeleopTest extends OpMode {
    ElapsedTime runtime = new ElapsedTime();
    ROBOT robot = new ROBOT();
    @Override
    public void init()   {
        robot.init(new Position(0,0,0), hardwareMap, gamepad1, gamepad2, telemetry);
    }

    @Override
    public void loop() {
        robot.addTask(new Tasks(Tasks.taskType.TELEOP_PL1, Tasks.taskRunMode.HOTCAKE, null));
        robot.addTask(new Tasks(Tasks.taskType.TELEOP_PL2, Tasks.taskRunMode.HOTCAKE, null));
        robot.executeTasks();
    }

}
