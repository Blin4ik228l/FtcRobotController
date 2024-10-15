package org.firstinspires.ftc.teamcode.RobotCore.RobotUtils;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RobotCore.RobotSubsystems.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.RobotCore.RobotSubsystems.Odometry;
import org.firstinspires.ftc.teamcode.RobotCore.RobotSubsystems.Subsystem;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.StdArgs;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.TaskHandler;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.TaskManager;
import org.firstinspires.ftc.teamcode.Utils.CONSTS;
import org.firstinspires.ftc.teamcode.Utils.PID;
import org.firstinspires.ftc.teamcode.Utils.Vector2;

public abstract class RobotCore implements Subsystem {

//  ПОЛЯ КЛАССА
////////////////////////////////////////////////////////////////////////////////////////////////////

    public RobotAlliance robotAlliance;
    public RobotMode robotMode;
    public OpMode opMode;

    // Менеджер задач робота
    public final TaskManager taskManager;

//  МЕТОДЫ КЛАССА
////////////////////////////////////////////////////////////////////////////////////////////////////

    // Конструктор класса
    public RobotCore(RobotMode robotMode, RobotAlliance robotAlliance, OpMode opMode) {
        this.robotAlliance = robotAlliance;
        this.robotMode = robotMode;
        this.opMode = opMode;

        taskManager = new TaskManager(this);
    }

//  МЕТОДЫ, ОБРАБАТЫВАЮЩИЕ ЗАДАЧИ
////////////////////////////////////////////////////////////////////////////////////////////////////

    /** Как написать свой метод-обработчик задачи?
     *
     */
    public TaskHandler example = new TaskHandler() {
        // Переменные, которые должны хранить долгосрочные данные в процессе
        // выполнения задачи нужно прописывать здесь.
        double var1;
        int var2;

        @Override
        public int execute(TaskManager thisTaskManager, StdArgs _args) {
            // Какое-то действие робота
            return -1;
        }
    };

    // Метод, обрабатывающий задачу телеопа
    public void teleop() {
        teleopPl1();
        teleopPl2();
    }

    public abstract void teleopPl1();
    public abstract void teleopPl2();

}

