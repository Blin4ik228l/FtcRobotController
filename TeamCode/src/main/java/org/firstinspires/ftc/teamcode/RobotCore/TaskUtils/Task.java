package org.firstinspires.ftc.teamcode.RobotCore.TaskUtils;

//Класс, описывающий структуру задачи, передаваемой в робота
public class Task {
    public Task(TaskCallback taskCallback, StdArgs args, taskRunMode runMode){
        this.taskCallback = taskCallback;
        this.runMode = runMode;
        this.args = args;
    }

    public TaskCallback taskCallback;
    public taskRunMode runMode;
    public StdArgs args;

    // Энам, перечисляющий режим начала выполнения задачи
    public enum taskRunMode {
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
