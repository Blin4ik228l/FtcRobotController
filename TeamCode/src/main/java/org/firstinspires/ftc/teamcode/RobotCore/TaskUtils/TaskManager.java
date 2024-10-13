package org.firstinspires.ftc.teamcode.RobotCore.TaskUtils;

import org.firstinspires.ftc.teamcode.RobotCore.RobotSubsystems.RobotCore;

import java.util.ArrayDeque;
import java.util.Deque;
import java.util.Iterator;
import java.util.Stack;

public class TaskManager {
////////////////////////////////////////////////////////////////////////////////////////////////////
    RobotCore robot;

    private final Deque<Task> taskDeque; // Двусторонняя очередь, содержащая задачи для выполнения
    private final Stack<Task> completedTasks; // Стэк для хранения выполненных задач

    TaskExecMode currentTaskExecMode = TaskExecMode.NONDEFINED;

////////////////////////////////////////////////////////////////////////////////////////////////////

    public TaskManager(TaskExecMode taskExecMode, RobotCore robot) {
        this.robot = robot;
        this.taskDeque = new ArrayDeque<Task>();
        this.completedTasks = new Stack<Task>();
        this.currentTaskExecMode = taskExecMode;
    }

    public boolean isTeleopMode() {
        return currentTaskExecMode == TaskExecMode.TELEOP;
    }

    public boolean isAutoMode() {
        return currentTaskExecMode == TaskExecMode.AUTO;
    }

    /** Обработчик задач
     * Этот метод предназначен для обработки задач, содержащихся в taskDeQueue.
     * Суть такого подхода заключается в том, что программист в классе Task добавляет в
     * taskType задачи, которые робот должен выполнять, в taskRunMode добавляет режимы,
     * в которых задачи могут выполняться и реализует методы, которые обеспечивают выполнение задачи.
     * Преимущество такой организации выполнения программы заключается в том, что задачи в очередь
     * можно добавлять из любого места программы, в любой момент, в начало и конец очереди,
     * а задача обработчика - выполнить задачи в нужной последовательности.
     * Как этим пользоваться?
     * Создать объект класса ROBOT в нужной программе, инициализировать робота
     * В программе с автономкой или телеопом, например, в методе runOpMode
     * добавить роботу задачу:
     *          Args.driveArgs args = new Args.driveArgs(new Position(12,16,0), 20);
     *          ROBOT.Task new_task = new ROBOT.Task(   ROBOT.taskType.DRIVE_TO_POSITION,
     *                                                  ROBOT.taskRunMode.START_AFTER_PREVIOUS,
     *                                                  args);
     *          my_robot.addTask(new_task);
     * Задач можно добавлять сколько угодно и какие угодно.
     * Для начала выполнения задач в конце напишите
     *      my_robot.executeTasks();
     */
    public void start(){
        // Очередь для хранения обрабатываемых задач
        Deque<Task> executingTasks = new ArrayDeque<>();

        // Обработчик будет работать, пока есть задачи
        while(!taskDeque.isEmpty() || this.isTeleopMode()) {
            /*
                Стартер задач
                Если taskRunMode первой задачи в очереди taskDeQueue соответствует условиям,
                то задача переходит в очередь обрабатываемых задач.
                В этом ветвлении программист должен прописать логику обработки runMode'ов задач.
                Задачи из taskDeQueue стоит добавлять в конец очереди executingTasks, разве что
                если вы полностью уверены в том, что делаете
            */
            switch (taskDeque.getFirst().runMode){
                // Пример:
                //  case ВАШ_ЭНАМ_ИЗ_taskRunMode
                //      assert executingTasks != null;
                //      какая-то логика, переносящая задачу из taskDeQueue в executingTasks
                //      для начала обработки задачи.
                //      break; <- обязательно

                case START_AFTER_PREVIOUS:
                    if(executingTasks.isEmpty()){
                        executingTasks.addLast(taskDeque.getFirst());
                        taskDeque.removeFirst();
                    }
                    break;

                case START_WITH_PREVIOUS:
                    executingTasks.addLast(taskDeque.getFirst());
                    taskDeque.removeFirst();
                    break;

                case HOTCAKE:
                    executingTasks.addFirst(taskDeque.getFirst());
                    taskDeque.removeFirst();
                    break;
            }

            /*
                Итератор задач, которые лежат в очереди выполняемых задач executingTasks.
                Проходим по каждой задаче и выполняем метод, который предназначен для выполнения задачи.
                ВАЖНО! Внутри метода не должно быть циклов, ожидающих выполнения задачи, метод проходится один
                раз за итерацию итератора task, это позволяет дергать методы-обработчики в цикле итератора
                несколько раз в секунду и работать над несколькими задачами "одновременно".
            */

            Iterator<Task> task = executingTasks.iterator();
            while ( task.hasNext() || this.isTeleopMode()) {
                Task currentTask = task.next();

                if (this.isTeleopMode()) {
                    robot.teleop();
                }

                /*
                    Выбор метода, который должен отработать.
                    Здесь программист должен указать, какому методу должна соответствовать задача,
                    имеющая определенный taskType.
                    Обратите внимание, что ваш метод должен возвращать int, показывая результат своей работы,
                    который нужно присвоить переменной result.
                    Если result = -1, программа продолжает выполнение
                    Если result = 0, значит робот выполнил задачу и она переносится из очереди в стэк
                    completedTasks, в котором хранятся выполненные задачи.
                 */

                StdArgs _args = currentTask.args;
                int result = currentTask.taskCallback.execute(this, _args);

                // Условие завершения выполнения задачи
                switch (result){
                    case -1:
                        break;
                    case 0:
                        completedTasks.push(currentTask);
                        task.remove();
                        break;
                }
                /*
                    Если выполнился обработчик задачи с режимом HOTCAKE, то итерация не продолжается,
                    а начинается с начала. Это позволяет выполнять обработчику только текущую HOTCAKE
                    задачу.
                 */
                if (currentTask.runMode == Task.taskRunMode.HOTCAKE) {
                    break;
                }
            }
        }
    }

    // Добавление задачи в конец очереди
    public void addTask(Task newtask) {
        taskDeque.addLast(newtask);
    }

    // Добавление задачи в начало очереди
    public void addTaskFirst(Task newtask) {
        taskDeque.addFirst(newtask);
    }
}
