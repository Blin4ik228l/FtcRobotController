package org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.Tasks;

import org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.States;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.StandartArgs;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.TaskHandlers.TaskHandlerOrdinal;

public class OrdinaryTask {

    public OrdinaryTask(TaskHandlerOrdinal taskHandler, StandartArgs args, taskStartMode startMode){
        this.taskHandler = taskHandler;
        this.startMode = startMode;
        this.reward = reward;
        this.args = args;
        this.nameTask = nameTask;

        this.state = States.TODO;
    }

    public TaskHandlerOrdinal taskHandler;
    public String nameTask;
    public int reward;
    public OrdinaryTask.taskStartMode startMode;
    public StandartArgs args;

    public States state;
    public double startTime; // Время начала выполнения задачи относительно runtime TaskManager'а
    public double finishTime;

    public double progressBar;


    // Энам, перечисляющий режим начала выполнения задачи
    public enum taskStartMode {
        // Режим HOTCAKE предназначен для задач, которые необходимо выполнить
        // немедленно, приостановив выполнение всех остальных задач.
        // Такой режим может пригодиться, если робот поймет, что он застрял
        // или выполнение задачи по каким то причинам стало невозможно.
        // Для обработки таких проблем методы задач должны иметь в себе функционал,
        // позволяющий обнаружить ошибку и добавить задачу
        // с режимом HOTCAKE в НАЧАЛО очереди taskDeQueue
        // Тип задачи-фиксика тоже должен быть прописан в предыдущем энаме
        HOTCAKE,
        // Задача с таким режимом запустится только после завершения всех предыдущих задач
        START_AFTER_PREVIOUS,
        // Задача с таким режимом начнет выполнение вместе с предыдущей
        START_WITH_PREVIOUS
    }
}
