package org.firstinspires.ftc.teamcode.RobotCore.TaskUtils;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.States;
import org.firstinspires.ftc.teamcode.RobotCore.RobotCore;
import org.firstinspires.ftc.teamcode.RobotCore.RobotUtils.RobotMode;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.Tasks.OrdinaryTask;

import java.util.ArrayDeque;
import java.util.Deque;
import java.util.Iterator;
import java.util.Stack;

public class TaskManager extends Thread{
////////////////////////////////////////////////////////////////////////////////////////////////////

    private RobotCore robot; // Храним здесь объект, который владеет TaskManager'ом

//    private final Robot robot;
    private final ElapsedTime managerRuntime; // Рантайм объекта TaskManager
    private final Deque<OrdinaryTask> taskDeque; // Двусторонняя очередь, содержащая задачи для выполнения
    private final Deque<OrdinaryTask> executingDeque; // Очередь для хранения обрабатываемых задач
    private final Stack<OrdinaryTask> completedTasks; // Стэк для хранения выполненных задач
    private Stack<OrdinaryTask> tasksToDo;

    public boolean goNextTask = false;

    public OrdinaryTask taskFromNode;

////////////////////////////////////////////////////////////////////////////////////////////////////

    public TaskManager(RobotCore robot) {
        this.robot = robot;

        this.managerRuntime = new ElapsedTime();
        this.taskDeque = new ArrayDeque<OrdinaryTask>();
        this.executingDeque = new ArrayDeque<OrdinaryTask>();
        this.completedTasks = new Stack<OrdinaryTask>();
    }


    /** Обработчик задач
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

     *   MyArgs.МойКлассАргумент _args = new MyArgs.МойКлассАргумент( Ваши параметры );
     *   Task _task = new Task(my_robot.МойОбработчикЗадачи, _args, Task.taskStartMode.УСЛОВИЕ_НАЧАЛА_ВЫПОЛНЕНИЯ);
     *   my_robot.taskManager.addTask(_task);

     * Задач можно добавлять сколько угодно.
     * Для начала выполнения задач в методе runOpMode напишите
     *      my_robot.taskManager.start();
     */


    public void forAuto(){
        if(!executingDeque.isEmpty() || !taskDeque.isEmpty()){
            pickTaskToDo();

            taskHandler();
        }
    }

    public void forTeleop(){
        this.setDaemon(true);
        this.start();
    }

    @Override
    public void run() {
        while (this.isAlive()) {
            startTasks();
        }
    }

    private synchronized void startTasks() {
        // Обработчик будет работать, пока есть задачи либо пока робот в телеоп режиме
        while(!isStopMode()) {
            if(isTeleopMode()){
                robot.teleop();
            }
        }
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


    public void permanentlyExecute(){
        stackTasks();

        OrdinaryTask executingTask = prepareTask();

        executeTask(executingTask);

    }

    public OrdinaryTask prepareTask(){

        OrdinaryTask t = tasksToDo.peek();

        if(t.state == States.TODO){
            t.startTime = managerRuntime.milliseconds();
            t.state = States.RUNNING;
            t.taskHandler.init(this, t.args);
        }else return t;

        return t;
    }

    public void executeTask(OrdinaryTask executingTask){

        int result = updateTask(executingTask);

        // Вернулся 0 - значит задача выполнилась
        if (result == 0 || goNextTask) {
            goNextTask = false;
            // Выкидываем задачу в стэк выполненного
            completedTasks.push(executingTask);

            taskDeque.remove(executingTask);//Убираем из общей очереди задач
            tasksToDo.pop();//Убираем из очереди выполняемых задач
        }

    }

    public void addTaskToStack(OrdinaryTask newTask){
        if(newTask.startMode == OrdinaryTask.taskStartMode.START_AFTER_PREVIOUS) taskDeque.addFirst(newTask);

        if(newTask.startMode == OrdinaryTask.taskStartMode.HOTCAKE) taskDeque.addLast(newTask);
    }

    public void stackTasks(){
        tasksToDo = new Stack<>();

        for (Iterator<OrdinaryTask> iterator = taskDeque.iterator(); iterator.hasNext() ; ) {
            OrdinaryTask currentTask = iterator.next();

            tasksToDo.push(currentTask);
        }
    }


    /**
     * Стартер задач.
     * Если taskStartMode первой задачи в очереди taskDeque соответствует условиям,
     * то задача переходит в очередь обрабатываемых задач,
     * получает время начала выполнения и режим - DOING.
     */
    private void pickTaskToDo()
    {
        boolean picked = true; // Тут храним результат выбора
        if (!taskDeque.isEmpty()) {
            /*
                В этом ветвлении программист должен прописать логику обработки startMode'ов задач.
                Задачи из taskDeque стоит добавлять в конец очереди executingDeque, разве что
                если вы полностью уверены в том, что делаете
             */
            switch (taskDeque.getFirst().startMode) {
                // Пример:
                //  case ВАШ_ЭНАМ_ИЗ_taskStartMode
                //      какое-то условие для переноса задачи из taskDeque в executingDeque
                //      для начала обработки задачи.
                //          pollToLast(); или pollToFirst();
                //      break; <- обязательно

                case START_AFTER_PREVIOUS:
                    if (executingDeque.isEmpty()) {
                        pollToLast();
                    } else if (executingDeque.getLast().state == States.SUCCESS) {
                        pollToLast();
                    } else {
                        picked = false; // Если не перенесли
                    }
                    break;

                case START_WITH_PREVIOUS:
                    pollToLast();
                    break;

                case HOTCAKE:
                    pollToFirst();
                    break;

                default:
                    picked = false; // Если не попали ни в один из кейсов
                    break;
            }
        } else {
            picked = false;
        }

        // Если перенесли, то обновляем состояние на DOING и задаем время старта выполнения задачи
        if (picked) {
            OrdinaryTask t = executingDeque.getFirst();
            t.startTime = managerRuntime.milliseconds();
            t.state = States.RUNNING;
            t.taskHandler.init(this, t.args);
        }
    }
    /**
     * Итератор задач, которые лежат в очереди выполняемых задач executingTasks.
     * Проходим по каждой задаче и обновляем ее, принимая результат обновления.
     */
    private void taskHandler() {
        for (Iterator<OrdinaryTask> iterator = executingDeque.iterator(); iterator.hasNext() ; ) {
            OrdinaryTask currentTask = iterator.next();

            int result = updateTask(currentTask);

            // Вернулся 0 - значит задача выполнилась
            if (result == 0 || goNextTask) {
                goNextTask = false;
                // Проверяем, можно ли теперь взять новую задачу
                pickTaskToDo();
                // Выкидываем задачу в стэк выполненного
                completedTasks.push(currentTask);
                iterator.remove();
            }
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
            while (result != 0 || goNextTask) {
                result = currentTask.taskHandler.execute(this, currentTask.args);
            }
        } else {
            result = currentTask.taskHandler.execute(this, currentTask.args);
        }

        // Вернулся 0 -> задача выполнена -> ставим ее в режим DONE
        if (result == 0 || goNextTask) {
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

    public boolean isTeleopMode() {
        return robot.robotMode == RobotMode.TELEOP;
    }
    public boolean isAutoMode() {
        return robot.robotMode == RobotMode.AUTO;
    }
    public boolean isStopMode(){
        return robot.robotMode == RobotMode.STOP;
    }

    public void getTasksInDeque (){
        for (int i = 0; i < taskDeque.size(); i++) {

        }
        robot.op.telemetry.addLine("TaskInDeque")
                .addData("TaskInDeque",taskDeque.getFirst().nameTask);

    }


}
