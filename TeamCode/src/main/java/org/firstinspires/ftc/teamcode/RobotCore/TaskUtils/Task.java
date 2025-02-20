package org.firstinspires.ftc.teamcode.RobotCore.TaskUtils;

//Класс, описывающий структуру задачи, передаваемой в робота
public class Task {
    public Task(TaskHandler taskHandler, StandartArgs args, int reward, taskStartMode startMode){
        this.taskHandler = taskHandler;
        this.startMode = startMode;
        this.reward = reward;
        this.args = args;
        this.nameTask = nameTask;

        this.state = States.TODO;
    }

    public String nameTask;
    public int reward;
    public TaskHandler taskHandler;
    public taskStartMode startMode;
    public StandartArgs args;

    public States state;
    public double startTime; // Время начала выполнения задачи относительно runtime TaskManager'а
    public double finishTime;

    public double progressBar;

    public enum States {
        TODO,
        DOING,
        DONE
    }

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
