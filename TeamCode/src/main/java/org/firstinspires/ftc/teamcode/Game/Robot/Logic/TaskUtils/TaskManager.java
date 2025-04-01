package org.firstinspires.ftc.teamcode.Game.Robot.Logic.TaskUtils;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Game.Robot.Logic.BehaviorTree.States;
import org.firstinspires.ftc.teamcode.Game.Robot.Logic.TaskUtils.Tasks.OrdinaryTask;
import org.firstinspires.ftc.teamcode.Game.Robot.RobotCore;

import java.util.ArrayDeque;
import java.util.Deque;
import java.util.Iterator;
import java.util.Stack;

public class TaskManager{
////////////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * Обработчик задач
     * Этот метод предназначен для обработки задач, содержащихся в taskDeque.
     * Суть такого подхода заключается в том, что программист создает класс робота,
     * который наследуется от RobotCore, на основе интерфейса TaskHandler реализует методы,
     * которые обеспечивают выполнение задачи.
     * Преимущество такой организации выполнения программы заключается в том, что задачи в очередь
     * можно добавлять из любого места программы, в любой момент, в начало и конец очереди,
     * а задача обработчика - выполнить задачи в нужной последовательности.
     * Как этим пользоваться?
     * Создать класс Robot, наследующий RobotCore, расписать в нем функционал вашего робота,
     * в файле с программой телеопа или автономки создать объект класса Robot.
     * В методе runOpMode или init или т.п. добавить роботу задачу:
     * <p>
     * MyArgs.МойКлассАргумент _args = new MyArgs.МойКлассАргумент( Ваши параметры );
     * Task _task = new Task(my_robot.МойОбработчикЗадачи, _args, Task.taskStartMode.УСЛОВИЕ_НАЧАЛА_ВЫПОЛНЕНИЯ);
     * my_robot.taskManager.addTask(_task);
     * <p>
     * Задач можно добавлять сколько угодно.
     * Для начала выполнения задач в методе runOpMode напишите
     * my_robot.taskManager.start();
     */
    private RobotCore robot; // Храним здесь объект, который владеет TaskManager'ом

    private final ElapsedTime managerRuntime; // Рантайм объекта TaskManager
    private final Deque<OrdinaryTask> taskDeque; // Двусторонняя очередь, содержащая задачи для выполнения
    private final Deque<OrdinaryTask> executingDeque; // Очередь для хранения обрабатываемых задач
    private final Stack<OrdinaryTask> completedTasks; // Стэк для хранения выполненных задач
    private Stack<OrdinaryTask> tasksToDo;
////////////////////////////////////////////////////////////////////////////////////////////////////

    public TaskManager(RobotCore robot) {
        this.robot = robot;

        this.managerRuntime = new ElapsedTime();
        this.taskDeque = new ArrayDeque<OrdinaryTask>();
        this.executingDeque = new ArrayDeque<OrdinaryTask>();
        this.completedTasks = new Stack<OrdinaryTask>();
    }

    public Deque<OrdinaryTask> getTaskDeque() {
        return taskDeque;
    }

    public Deque<OrdinaryTask> getExecutingDeque() {
        return executingDeque;
    }

    public Stack<OrdinaryTask> getCompletedTasks() {
        return completedTasks;
    }

    public void permanentlyExecute() {
        stackTasks();
        OrdinaryTask executingTask = prepareTask();
        executeTask(executingTask);
    }

    /**
     * Стартер задач.
     * Если taskStartMode первой задачи в очереди taskDeque соответствует условиям,
     * то задача переходит в очередь обрабатываемых задач,
     * получает время начала выполнения и режим - DOING.
     */
    public OrdinaryTask prepareTask(){

        OrdinaryTask t = tasksToDo.peek();

        if(t.state == States.TODO){
            t.startTime = managerRuntime.milliseconds();
            t.state = States.RUNNING;
            t.taskHandler.init(this, t.args);
        }else return t;

        return t;
    }

    public void executeTask(OrdinaryTask executingTask) {

        int result = updateTask(executingTask);

        if (executingTask.state == States.FAILURE){
            taskDeque.remove(executingTask);//Убираем из общей очереди задач
            tasksToDo.pop();//Убираем из очереди выполняемых задач
        }

        // Вернулся 0 - значит задача выполнилась
        if (result == 0 ) {
            // Выкидываем задачу в стэк выполненного
            completedTasks.push(executingTask);

            taskDeque.remove(executingTask);//Убираем из общей очереди задач
            tasksToDo.pop();//Убираем из очереди выполняемых задач
        }
    }

    public void addTaskToStack(OrdinaryTask newTask){
        if(newTask.startMode == OrdinaryTask.taskStartMode.START_AFTER_PREVIOUS ||
                newTask.startMode == OrdinaryTask.taskStartMode.START_WITH_PREVIOUS) taskDeque.addFirst(newTask);

        if(newTask.startMode == OrdinaryTask.taskStartMode.HOTCAKE) taskDeque.addLast(newTask);
    }

    public void stackTasks() {
        tasksToDo = new Stack<>();

        for (Iterator<OrdinaryTask> iterator = taskDeque.iterator(); iterator.hasNext(); ) {
            OrdinaryTask currentTask = iterator.next();

            tasksToDo.push(currentTask);
        }
    }
    /**
     * Выполняем метод, который предназначен для обработки задачи.
     * ВАЖНО! Внутри обработчика не должно быть циклов, ожидающих выполнения задачи, метод
     * проходится один раз за вызов, это позволяет дергать методы-обработчики в цикле Итератора
     * несколько раз в секунду и работать над несколькими задачами "одновременно".
     */
    private int updateTask(OrdinaryTask currentTask) {
        /*
            Обратите внимание, что ваш метод должен возвращать int, показывая результат своей работы,
            который нужно присвоить переменной result.
            Если result = -1, программа продолжает выполнение
            Если result = 0, задача отмечается как DONE
        */
        int result = -1;

        // Обработчики HOTCAKE задач крутятся в цикле, пока не выполнят задачу,
        // остальные обработчики вызываются один раз
        if (currentTask.startMode == OrdinaryTask.taskStartMode.HOTCAKE) {
            while (result != 0) {
                result = currentTask.taskHandler.execute(this, currentTask.args);
            }
        } else {
            result = currentTask.taskHandler.execute(this, currentTask.args);
        }

        // Вернулся 0 -> задача выполнена -> ставим ее в режим DONE
        if (result == 0) {
            currentTask.finishTime = managerRuntime.milliseconds();
            currentTask.state = States.SUCCESS;
        }

        // Возвращаем 0, если задача перешла в режим DONE
        return result;
    }

    // Перенести задачу из очереди задач в начало очереди исполняемых задач
    private void pollToFirst() {
        executingDeque.offerFirst(taskDeque.pollFirst());
    }

    // Перенести задачу из очереди задач в конец очереди исполняемых задач
    private void pollToLast() {
        executingDeque.offerLast(taskDeque.pollFirst());
    }
    // Добавление задачи в конец очереди
    public void addTask(OrdinaryTask newtask) {
        taskDeque.offerLast(newtask);
    }

    // Добавление задачи в начало очереди
    public void addTaskFirst(OrdinaryTask newtask) {
        taskDeque.offerFirst(newtask);
    }

    // Доделать последнюю выполненную задачу
    public void redoLastTask() {
        addTaskFirst(completedTasks.peek());
    }
}