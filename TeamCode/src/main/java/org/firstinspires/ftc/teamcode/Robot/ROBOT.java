package org.firstinspires.ftc.teamcode.Robot;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Robot.TaskHandler.Args;
import org.firstinspires.ftc.teamcode.Robot.TaskHandler.Tasks;
import org.firstinspires.ftc.teamcode.Utils.CONSTS;
import org.firstinspires.ftc.teamcode.Utils.PID;
import org.firstinspires.ftc.teamcode.Utils.Position;
import org.firstinspires.ftc.teamcode.Robot.RobotSubsystems.Odometry;
import org.firstinspires.ftc.teamcode.Utils.Vector2;

import java.util.Deque;
import java.util.Iterator;
import java.util.Stack;

public class ROBOT {
//  ПОЛЯ КЛАССА
////////////////////////////////////////////////////////////////////////////////////////////////////
    private DcMotorEx rightB, rightF, leftB, leftF;     // Объекты моторов
    private DcMotorEx encM, encL, encR;                 // Объекты энкодеров
    Odometry odometry;                                  // Объект, обрабатывающий одометрию

    private Deque<Tasks> taskDeQueue;                    // Двусторонняя очередь, содержащая задачи для выполнения
    private Stack<Tasks> completedTasks;                  // Стэк для хранения выполненных задач

    private PID pidDriveTrainLinear, pidDriveTrainAngular;

//  МЕТОДЫ КЛАССА
////////////////////////////////////////////////////////////////////////////////////////////////////
    // Метод инициализации того, чего надо
    public void init(Position startPosition){
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

        pidDriveTrainLinear.setPID(1,1,1);
        pidDriveTrainAngular.setPID(1,1,1);
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
        Deque<Tasks> executingTasks = null;
        // Обработчик будет работать, пока есть задачи
        while(!taskDeQueue.isEmpty()){

            /*
                Стартер задач
                Если taskRunMode первой задачи в очереди taskDeQueue соответствует условиям,
                то задача переходит в очередь обрабатываемых задач.
                В этом ветвлении программист должен прописать логику обработки runMode'ов задач.
                Задачи из taskDeQueue стоит добавлять в конец очереди executingTasks, разве что
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
            Iterator<Tasks> task = executingTasks.iterator();
            while ( task.hasNext() ) {
                Tasks currentTask = task.next();
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
                switch (currentTask.type){
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
                    case TELEOP_PL1:
                        //метод дял телеопа
                        result = driveTeleOp();
                        break;
                    case TELEOP_PL2:
                        result = teleskopeTeleOp();
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
                if (currentTask.runMode == Tasks.taskRunMode.HOTCAKE) {
                    break;
                }
            }
        }
    }



    // Добавление задачи в конец очереди
    public void addTask(Tasks newtask){
        taskDeQueue.addLast(newtask);
    }


    // Распределение требуемой скорости и направления движения робота на скорость колес
    private void setVelocity(Vector2 direct, double heading){
        // TODO
        rightF.setPower(Range.clip((-direct.x - direct.y - heading), -1.0, 1.0));
        leftB.setPower(Range.clip((direct.x + direct.y - heading), -1.0, 1.0));
        leftF.setPower(Range.clip((direct.x - direct.y - heading), -1.0, 1.0));
        rightB.setPower(Range.clip((-direct.x + direct.y - heading), -1.0, 1.0));
    }
    private void brakeMotors(){
        rightF.setPower(0);
        leftB.setPower(0);
        leftF.setPower(0);
        rightB.setPower(0);

        rightF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    void setVelocityTele(){

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
        int result;

        Vector2 errorPos = args.position.toVector();
        errorPos.sub(odometry.getGlobalPosition().toVector());

        Vector2 direction = new Vector2(errorPos);
        direction.normalize();

        double errorHeading = args.position.heading - odometry.getGlobalPosition().getHeading();
        double speedPID = pidDriveTrainLinear.calculate(args.max_linear_speed, odometry.getSpeed());

        double angularPID = pidDriveTrainAngular.calculate(args.max_angular_speed, odometry.getAngularVelocity());//Всегда положителен

        direction.multyplie(speedPID);
        errorHeading *= angularPID;

        setVelocity(direction, errorHeading);

        if(direction.x != odometry.getGlobalPosition().getX() && direction.y != odometry.getGlobalPosition().getY()
                && errorHeading != odometry.getGlobalPosition().getHeading()){
            result = -1;
        }else{
            brakeMotors();
            result = 0;
        }

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

    public int teleskopeTeleOp(){

        return -1;
    }

    public int driveTeleOp(){

        double velocityAngle = (((encL.getVelocity() + encR.getVelocity())/2)/CONSTS.TICK_PER_CM)/(CONSTS.DIST_BETWEEN_ENC_X/2);// см/сек
        double velocityX = (((encL.getVelocity() - encR.getVelocity())/2)/CONSTS.TICK_PER_CM);// см/сек
        double velocityY = (encM.getVelocity() - velocityAngle * CONSTS.OFFSET_ENC_M_FROM_CENTER);

        double xVel = gamepad1.left_stick_x * CONSTS.MAX_TPS_ENCODER/CONSTS.TICK_PER_CM;
        double yVel = gamepad1.left_stick_y * CONSTS.MAX_TPS_ENCODER/CONSTS.TICK_PER_CM;
        double headingVel = gamepad1.right_stick_x * CONSTS.MAX_TPS_ENCODER/CONSTS.TICK_PER_CM/(CONSTS.DIST_BETWEEN_ENC_X/2);

        double speed = 0.8;

        double kF = speed/CONSTS.MAX_CM_PER_SEC;//макс см/сек
        double kP = -0.0001;
        double kFR = speed/CONSTS.MAX_RAD_PER_SEC;//макс рад/сек

        double forward = (xVel - velocityX) * kP + xVel* kF;
        double side = (yVel - velocityY) * kP +  yVel* kF;
        double angle = (headingVel - velocityAngle) * kP +  headingVel* kFR;

        double rightFP = Range.clip((-forward - side - angle), -1.0, 1.0);
        double leftBP = Range.clip((forward + side - angle), -1.0, 1.0);
        double leftFP = Range.clip((forward - side - angle), -1.0, 1.0);
        double rightBP = Range.clip((-forward + side - angle), -1.0, 1.0);

        rightF.setPower(rightFP);
        leftB.setPower(leftBP);
        leftF.setPower(leftFP);
        rightB.setPower(rightBP);

        return -1;
    }
}

