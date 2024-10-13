package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.Deque;
import java.util.Iterator;
import java.util.Stack;

public class ROBOT {
//  ПОЛЯ КЛАССА
////////////////////////////////////////////////////////////////////////////////////////////////////
    private DcMotorEx rightB, rightF, leftB, leftF;     // Объекты моторов
    private DcMotorEx encM, encL, encR;                 // Объекты энкодеров
    Odometry odometry;                                  // Объект, обрабатывающий одометрию

    private Deque<Task> taskDeQueue;                    // Двусторонняя очередь, содержащая задачи для выполнения
    private Stack<Task> completedTasks;                  // Стэк для хранения выполненных задач

    private PID pid;

    // энам, перечисляющий возможные задачи робота, прописанные в программе
    enum taskType {
        // Ехать в указанную позицию
        DRIVE_TO_POSITION,
        // Выдвинуть телескоп на указанное значение
        SET_TELESKOPE_POS,
        STUCK_WHILE_DRIVING,
        STUCK_WHILE_UPPING_TELE
    }

    // Энам, перечисляющий режим начала выполнения задачи
    enum taskRunMode {
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


    // Подкласс, описывающий структуру задачи, передаваемой в робота
    public static class Task{
        Task(taskType taskType, taskRunMode runMode, Args args){
            this.taskType = taskType;
            this.runMode = runMode;
            this.args = args;
        }

        ROBOT.taskType taskType;
        taskRunMode runMode;
        Args args;
    }

//  МЕТОДЫ КЛАССА
////////////////////////////////////////////////////////////////////////////////////////////////////
    // Метод инициализации того, чего надо
    void init(Position startPosition){
        rightB = hardwareMap.get(DcMotorEx.class, "rightB");
        rightF = hardwareMap.get(DcMotorEx.class, "rightF");
        leftB = hardwareMap.get(DcMotorEx.class, "leftB");
        leftF = hardwareMap.get(DcMotorEx.class, "leftF");

        encM = hardwareMap.get(DcMotorEx.class, "encM") ;
        encL = hardwareMap.get(DcMotorEx.class, "encL") ;
        encR =  hardwareMap.get(DcMotorEx.class, "encR");

        rightB.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightF.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftF.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftB.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        encR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        odometry = new Odometry(startPosition, encL,  encR, encM);
        odometry.run();

        taskDeQueue = null;
        completedTasks = null;
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
    public void executeTasks(){
        // Очередь для хранения обрабатываемых задач
        Deque<Task> executingTasks = null;
        // Обработчик будет работать, пока есть задачи
        while(!taskDeQueue.isEmpty()){

            /*
                Стартер задач
                Если taskRunMode первой задачи в очереди taskDeQueue соответствует условиям,
                то задача переходит в очередь обрабатываемых задач.
                В этом ветвлении программист должен прописать логику обработки runMode'ов задач.
                Задачи из taskDeGueue стоит добавлять в конец очереди executingTasks, разве что
                если вы полностью уверены в том, что делаете
            */
            switch (taskDeQueue.getFirst().runMode){
            // Пример:
            //  case ВАШ_ЭНАМ_ИЗ_taskRunMode
            //      assert executingTasks != null;
            //      какая-то логика, переносящая задачу из taskDeQueue в executingTasks
            //      для начала обработки задачи.
            //      break; <- обязательно

                case START_AFTER_PREVIOUS:
                    assert executingTasks != null;
                    if(executingTasks.isEmpty()){
                        executingTasks.addLast(taskDeQueue.getFirst());
                        taskDeQueue.removeFirst();
                    }
                    break;

                case START_WITH_PREVIOUS:
                    assert executingTasks != null;
                    executingTasks.addLast(taskDeQueue.getFirst());
                    taskDeQueue.removeFirst();
                    break;

                case HOTCAKE:
                    assert executingTasks != null;
                    executingTasks.addFirst(taskDeQueue.getFirst());
                    taskDeQueue.removeFirst();
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
            while ( task.hasNext() ) {
                Task currentTask = task.next();
                int result;

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
                switch (currentTask.taskType){
                // Пример:
                //  case ВАШ_ЭНАМ_ИЗ_taskType:
                //      result = ваш метод, который предназначен для обработки задачи
                //      ВАЖНО! метод должен возвращать -1, если задача не выполнена до конца
                //      и 0, если задача выполнена.
                //      ВАЖНО!!! прочитайте, как правильно писать методы в файле Args.java
                //      break; <- обязательно!

                    case SET_TELESKOPE_POS:
                        result = setTeleskopePos(currentTask.args);
                        break;

                    case DRIVE_TO_POSITION:
                        result = driveToPosition(currentTask.args);
                        break;

                    // Выполняется, если задача не нашла своего обработчика
                    default:
                        result = 0;
                        break;

                }

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
                if (currentTask.runMode == taskRunMode.HOTCAKE) {
                    break;
                }
            }
        }
    }

    // Добавление задачи в конец очереди
    public void addTask(Task newtask){
        taskDeQueue.addLast(newtask);
    }

    // Распределение требуемой скорости и направления движения робота на скорость колес
    private void setVelocity(){
        // TODO
    }

//  МЕТОДЫ, ОБРАБАТЫВАЮЩИЕ ЗАДАЧИ
////////////////////////////////////////////////////////////////////////////////////////////////////

    /** Как написать свой метод-обработчик задачи?
     * Метод, обрабатывающий задачу должен иметь вид:
     * private int мойМетодОбработчик(Args _args){
     *     Args.мойКлассАргументов args = (Args.мойКлассАргументов)_args
     *     ...
     *     return result; // переменная result должна содержать 0, если задача завершена, иначе -1
     * }
     * в методе уже можете пользоваться переменной args как хотите
     */

    private int driveToPosition(Args _args){
        Args.driveArgs args = (Args.driveArgs) _args;
        //Найти угол между heading робота и направлением куда ехать

        int result = -1;
        return result;
    }

    private int setTeleskopePos(Args _args){
        Args.teleskopeArgs args = (Args.teleskopeArgs) _args;
        int result = -1;

        // TODO: обработчик застреваний телескопа
        //  если робот вдруг поехал
        //  если телескоп не поднялся на нужный уровень и стоит на месте долго

        return result;
    }
}

