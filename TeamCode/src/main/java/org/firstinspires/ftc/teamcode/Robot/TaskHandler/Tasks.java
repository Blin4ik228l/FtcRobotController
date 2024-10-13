package org.firstinspires.ftc.teamcode.Robot.TaskHandler;

import org.firstinspires.ftc.teamcode.Robot.ROBOT;

import java.util.Deque;
import java.util.Iterator;
//Класс, описывающий структуру задачи, передаваемой в робота
public class Tasks {
    public Tasks(taskType type, taskRunMode runMode, Args args){
        this.type = type;
        this.runMode = runMode;
        this.args = args;
    }

    public taskType type;
    public taskRunMode runMode;
    public Args args;

    // энам, перечисляющий возможные задачи робота, прописанные в программе
   public  enum taskType {
        // Ехать в указанную позицию
        DRIVE_TO_POSITION,
        // Выдвинуть телескоп на указанное значение
        SET_TELESKOPE_POS,
        STUCK_WHILE_DRIVING,
        STUCK_WHILE_UPPING_TELE,
        TELEOP_PL1,
        TELEOP_PL2
    }

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
