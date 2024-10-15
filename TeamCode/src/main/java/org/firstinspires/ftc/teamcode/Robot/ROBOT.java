package org.firstinspires.ftc.teamcode.Robot;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import android.content.Context;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.TaskHandler.Args;
import org.firstinspires.ftc.teamcode.Robot.TaskHandler.Tasks;
import org.firstinspires.ftc.teamcode.Utils.CONSTS;
import org.firstinspires.ftc.teamcode.Utils.PID;
import org.firstinspires.ftc.teamcode.Utils.Position;
import org.firstinspires.ftc.teamcode.Robot.RobotSubsystems.Odometry;
import org.firstinspires.ftc.teamcode.Utils.Vector2;

import java.util.ArrayDeque;
import java.util.Collection;
import java.util.Deque;
import java.util.Iterator;
import java.util.Stack;

public class ROBOT{
//  ПОЛЯ КЛАССА
////////////////////////////////////////////////////////////////////////////////////////////////////
    private DcMotor rightB,leftB;     // Объекты моторов
    private DcMotorEx encM, encL, encR, rightF, leftF;                 // Объекты энкодеров
    Odometry odometry;                                  // Объект, обрабатывающий одометрию

    private ArrayDeque<Tasks> taskDeQueue;
                   // Двусторонняя очередь, содержащая задачи для выполнения
    private Stack<Tasks> completedTasks;                                        // Стэк для хранения выполненных задач

    private ArrayDeque<Tasks> executingTasks;
    private PID pidDriveTrainLinear = new PID(1,1,1),
            pidDriveTrainAngular = new PID(1,1,1);
    Gamepad gamepad1, gamepad2;
    Telemetry telemetry;
//  МЕТОДЫ КЛАССА
////////////////////////////////////////////////////////////////////////////////////////////////////
    // Метод инициализации того, чего надо
    public void init(Position startPosition, HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry){

        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.telemetry = telemetry;

        rightB = hardwareMap.get(DcMotor.class, "rightB");
        rightF = hardwareMap.get(DcMotorEx.class, "rightF");
        leftB = hardwareMap.get(DcMotor.class, "leftB");
        leftF = hardwareMap.get(DcMotorEx.class, "leftF");

        encM = hardwareMap.get(DcMotorEx.class, "encM") ;
        encL = hardwareMap.get(DcMotorEx.class, "leftB") ;
        encR = hardwareMap.get(DcMotorEx.class, "rightB");

        rightB.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightF.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftF.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftB.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        encR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        encL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        encM.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftF.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightF.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        leftF.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        encM.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        encR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        encL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightF.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        odometry = new Odometry(startPosition, encL,  encR, encM, telemetry);
        odometry.start();

        executingTasks = new ArrayDeque<>();
        taskDeQueue = new ArrayDeque<>();
        completedTasks = new Stack<>();
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

    public void executeTasks() throws NullPointerException {
        // Очередь для хранения обрабатываемых задач

        try{
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
                while ( task.hasNext()) {
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
                    // Обработчик будет работать, пока есть задачи
                    if (currentTask.runMode == Tasks.taskRunMode.HOTCAKE) {
                        break;
                    }
                }
            }

        }catch (Exception e){
            e.getMessage();
        }


    }

    // Добавление задачи в конец очереди
    public void addTask(Tasks newtask) throws NullPointerException {
        try {
            taskDeQueue.addFirst(newtask);
        }catch (Exception e){
            e.getMessage();
        }


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

        double forward = (yVel - velocityX) * kP + yVel* kF;
        double side = (xVel - velocityY) * kP +  xVel* kF;
        double angle = (headingVel - velocityAngle) * kP +  headingVel* kFR;

        double rightFP = Range.clip((-forward - side - angle), -1.0, 1.0);
        double leftBP = Range.clip((forward + side - angle), -1.0, 1.0);
        double leftFP = Range.clip((forward - side - angle), -1.0, 1.0);
        double rightBP = Range.clip((-forward + side - angle), -1.0, 1.0);

        rightF.setPower(rightFP);
        leftB.setPower(leftBP);
        leftF.setPower(leftFP);
        rightB.setPower(rightBP);

        if(gamepad1.b){
            telemetry.addData("Напряга в rightB", rightB.getPower());
            telemetry.addData("Напряга в rightF", rightF.getPower());
            telemetry.addData("Напряга в leftF", leftF.getPower());
            telemetry.addData("Напряга в leftB", leftB.getPower());
            telemetry.addLine("\\");
            telemetry.addData("Скорость робота по X см/сек", velocityX );
            telemetry.addData("Скорость робота по Y см/сек", velocityY );
            telemetry.addData("Угловая скорость робота градус/сек",velocityAngle * (180/Math.PI));
            telemetry.addLine("\\");
            telemetry.addData("Скорость робота по X по джойстикам см/сек", xVel );
            telemetry.addData("Скорость робота по Y по джойстикам см/сек", yVel );
            telemetry.addData("Угловая скорость робота по джойстикам град/сек",headingVel * (180/Math.PI));
            telemetry.addLine("\\");
            telemetry.addData("Левый джойстик X", gamepad1.left_stick_x);
            telemetry.addData("Левый джойстик Y", gamepad1.left_stick_y);
            telemetry.addData("Правый джойстик X",gamepad1.right_stick_x);
            telemetry.addLine("\\");
        }else  {
//            telemetry.addData("leftF тики", leftB.getCurrentPosition());
//            telemetry.addData("Левый экодер тики", encL.getCurrentPosition());
//            telemetry.addData("Правый энкодер тики", encR.getCurrentPosition());
//            telemetry.addData("Серединный энкодер", encM.getCurrentPosition());
//            telemetry.addLine("\\");
//            telemetry.addData("Левый экодер см", encL.getCurrentPosition() / (CONSTS.TICK_PER_CM));
//            telemetry.addData("Правый энкодер см", encR.getCurrentPosition() / (CONSTS.TICK_PER_CM));
//            telemetry.addData("Серединный энкодер см", encM.getCurrentPosition() / (CONSTS.TICK_PER_CM));
//            telemetry.addLine("\\");
//            telemetry.addData("Левый экодер градус/сек", encL.getVelocity()/CONSTS.TICK_PER_DEGREES);
//            telemetry.addData("Правый энкодер градус/сек", encR.getVelocity()/CONSTS.TICK_PER_DEGREES);
//            telemetry.addData("Серединный градус/сек", encM.getVelocity()/CONSTS.TICK_PER_DEGREES);
//            telemetry.addLine("\\");
//            telemetry.addData("Угол робота", angle_robot);
//            telemetry.addData("Угол синус через рад", Math.sin(Rad));
//            telemetry.addData("Угол синус через град", Math.sin(angle_robot));
//            telemetry.addData("Радиан", Rad);
//            telemetry.addLine("\\");
//            telemetry.addData("Rx", Rx);
//            telemetry.addData("Ry", Ry);
//            telemetry.addData("Gx", Gx);
//            telemetry.addData("Gy", Gy);
//            telemetry.addLine("\\");
        }

        return -1;
    }
}

