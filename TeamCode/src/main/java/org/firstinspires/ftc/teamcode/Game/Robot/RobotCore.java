package org.firstinspires.ftc.teamcode.Game.Robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Game.Robot.Modules.Module;
import org.firstinspires.ftc.teamcode.Game.Robot.Modules.Unused.RobotStatusHandler;
import org.firstinspires.ftc.teamcode.Game.Robot.Logic.RobotUtils.RobotAlliance;
import org.firstinspires.ftc.teamcode.Game.Robot.Logic.RobotUtils.RobotMode;
import org.firstinspires.ftc.teamcode.Game.Robot.Logic.TaskUtils.StandartArgs;
import org.firstinspires.ftc.teamcode.Game.Robot.Logic.TaskUtils.TaskHandlers.TaskHandlerOrdinal;
import org.firstinspires.ftc.teamcode.Game.Robot.Logic.TaskUtils.TaskManager;

public abstract class RobotCore implements Module {

//  ПОЛЯ КЛАССА
////////////////////////////////////////////////////////////////////////////////////////////////////

    public RobotAlliance robotAlliance;
    public RobotMode robotMode;
    public OpMode op;

    // Менеджер задач робота
    public final TaskManager taskManager;

    public final RobotStatusHandler robotStatusHandler;

//  МЕТОДЫ КЛАССА
////////////////////////////////////////////////////////////////////////////////////////////////////

    // Конструктор класса
    public RobotCore(RobotMode robotMode, RobotAlliance robotAlliance, OpMode op) {
        this.robotAlliance = robotAlliance;
        this.robotMode = robotMode;
        this.op = op;

        taskManager = new TaskManager(this);
        robotStatusHandler = new RobotStatusHandler(taskManager);
    }

//  МЕТОДЫ, ОБРАБАТЫВАЮЩИЕ ЗАДАЧИ
////////////////////////////////////////////////////////////////////////////////////////////////////

    /** Как написать свой метод-обработчик задачи?
     *
     */
    public TaskHandlerOrdinal example = new TaskHandlerOrdinal() {
        // Переменные, которые должны хранить долгосрочные данные в процессе
        // выполнения задачи нужно прописывать здесь.
        double var1;
        int var2;

        @Override
        public int init(TaskManager thisTaskManager, StandartArgs _args) {
            return 0;
        }

        @Override
        public int execute(TaskManager thisTaskManager, StandartArgs _args) {
            // Какое-то действие робота

            return -1;
        }

    };}

