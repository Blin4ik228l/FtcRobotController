package org.firstinspires.ftc.teamcode.RobotCore;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.RobotCore.RobotSubsystems.Subsystem;
import org.firstinspires.ftc.teamcode.RobotCore.RobotUtils.RobotAlliance;
import org.firstinspires.ftc.teamcode.RobotCore.RobotUtils.RobotMode;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.StdArgs;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.TaskHandler;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.TaskManager;

public abstract class RobotCore implements Subsystem {

//  ПОЛЯ КЛАССА
////////////////////////////////////////////////////////////////////////////////////////////////////

    public RobotAlliance robotAlliance;
    public RobotMode robotMode;
    public OpMode op;

    // Менеджер задач робота
    public final TaskManager taskManager;

//  МЕТОДЫ КЛАССА
////////////////////////////////////////////////////////////////////////////////////////////////////

    // Конструктор класса
    public RobotCore(RobotMode robotMode, RobotAlliance robotAlliance, OpMode op) {
        this.robotAlliance = robotAlliance;
        this.robotMode = robotMode;
        this.op = op;

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

