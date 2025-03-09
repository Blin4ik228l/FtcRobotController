package org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules.Virtual;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.OpModes.Robot;
import org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.States;
import org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules.Module;
import org.firstinspires.ftc.teamcode.RobotCore.RobotModulesStatus.ComonStatuses.MotorsStatus;
import org.firstinspires.ftc.teamcode.RobotCore.RobotModulesStatus.OtherStatuses.RobotStatusInDrive;
import org.firstinspires.ftc.teamcode.RobotCore.RobotModulesStatus.OtherStatuses.TeleskopeStatusInMoving;
import org.firstinspires.ftc.teamcode.RobotCore.RobotModulesStatus.RobotModuleStatus;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.StandartArgs;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.TaskManager;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.Tasks.OrdinaryTask;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.Tasks.VarietiesTasks.DriveTask;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.Tasks.VarietiesTasks.TeleskopeTask;
import org.firstinspires.ftc.teamcode.RobotCore.Utils.Position;

import java.util.ArrayDeque;
import java.util.Deque;
import java.util.Iterator;
import java.util.Stack;

public class RobotStatusHandler extends Thread implements Module {
    public RobotStatusHandler(TaskManager taskManager){
        this.taskManager = taskManager;
    }

    public TaskManager taskManager;
    public Robot robot;

    public ElapsedTime runtime = new ElapsedTime();


    @Override
    public void init() {
        this.setDaemon(true);
    }

    @Override
    public void run() {
        while (this.isAlive()) startLurking();
    }

    public void loadTasks(){
        tasksToDo = taskManager.getTaskDeque();
    }

    public Deque<OrdinaryTask> tasksToDo = new ArrayDeque<>();

    public void startLurking() {
        while (!tasksToDo.isEmpty() && this.isAlive()) {
            while(taskManager.getExecutingDeque() == null && this.isAlive()){
                //..waiting
            }Deque<OrdinaryTask> executingTasks = taskManager.getExecutingDeque();

            for (Iterator<OrdinaryTask> iterator = executingTasks.iterator(); iterator.hasNext();) {
                OrdinaryTask currentTask = iterator.next();

                //Запускаем время с начала выполнения задачи
                runtime.seconds();
                if (currentTask.state == States.SUCCESS) {
                    runtime.reset();
                    iterator.remove();
                } else {
                    //Если время превысило n - ое кол - во секунд, тогда возвращаем статус FAILURE
                    if (runtime.seconds() > 5 && currentTask.taskHandler != robot.robotSleep) {
                        runtime.reset();
                        currentTask.state = States.FAILURE;
                        currentTask.startMode = OrdinaryTask.taskStartMode.HOTCAKE;
                    }
                    /*
                     * statusInDrive[]
                     * 0 элемент - статус по X
                     * 1 элемент - статус по Y
                     * 3 элемент - статус при повороте
                     * ---
                     * Элементы внутри statusInDrive[] расположены примерно так:
                     * statusInDrive[1] = {None, Moving, Completed}, (Это последовательность действий робота во время выполнения задачи)
                     * statusInDrive[1] = {None, MovingInOtherSide, Stucked}, (Это последовательность действий робота во время выполнения задачи)
                     */

                    //TODO: Дописать для каждого типа задачи(обработчика)
                    else if (currentTask instanceof DriveTask) {

                        DriveTask driveTask = (DriveTask) currentTask;//Текущие аргументы

                        Deque<RobotStatusInDrive> historyOfXmove = driveTask.driveHandler.statusInDrive()[0];
                        Deque<RobotStatusInDrive> historyOfYmove = driveTask.driveHandler.statusInDrive()[1];
                        Deque<RobotStatusInDrive> historyOfRotate = driveTask.driveHandler.statusInDrive()[2];

                        StandartArgs.driveArgs lastArgs = null;
                        Position lastPos = new Position();//Переменная для сохранения позиции из прошлой задачи

                        Stack<OrdinaryTask> completedTasksInManager = robot.taskManager.getCompletedTasks();//Не трогаем целостность очереди из выполненых задач

                        StandartArgs.driveArgs newArgs;

                        //Смотрим из очереди выполненых задач на прошлую выполненую задачу, чтобы например вернутся на прошлую позицию
                        if (completedTasksInManager.peek().taskHandler == robot.driveToPosition) {
                            lastArgs = (StandartArgs.driveArgs) completedTasksInManager.peek().args;
                        } else {
                            //Ищем в очереди выполненых задач, задачу того же типа, чтобы попытаться вернутья в прошлою позицию
                            while (completedTasksInManager.peek().taskHandler != robot.driveToPosition) {
                                completedTasksInManager.pop();

                                if (completedTasksInManager.peek().taskHandler == robot.driveToPosition) {
                                    lastArgs = (StandartArgs.driveArgs) completedTasksInManager.peek().args;
                                }

                                if (robot.taskManager.getCompletedTasks().isEmpty()) {
                                    lastArgs = new StandartArgs.driveArgs(robot.odometry.getStartGlobalPosition());
                                    break;
                                }
                            }
                        }

                        //TODO: Дописать для статуса Moving
                        /*--------------------Отладка движения по X-----------------------*/
                        if (historyOfXmove.getLast() == RobotStatusInDrive.StuckedBy_X) {
                            historyOfXmove.removeLast();
                            robot.drivetrain.driveTrainStatus = RobotModuleStatus.Stucked;

                            if (historyOfXmove.getLast() == RobotStatusInDrive.MovingInOtherSideBy_X) {
                                robot.drivetrain.motorsXdirection = MotorsStatus.Reversed;
                                lastPos.setX(lastArgs.position.getX());
                            }
                            if (historyOfXmove.getLast() == RobotStatusInDrive.MovingBy_X) {
                                robot.drivetrain.motorsXdirection = MotorsStatus.Normal;
                            }
                            lastPos.setX(lastArgs.position.getX());
                        } else if (historyOfXmove.getLast() == RobotStatusInDrive.MovingInOtherSideBy_X) {
                            robot.drivetrain.driveTrainStatus = RobotModuleStatus.Stucked;
                            lastPos.setX(lastArgs.position.getX());
                        } else {
                            //Остаётся статус Moving значит всё нормально
                            lastPos.setX(driveTask.args.position.getX());//Ничего не трогаем из прошлых аргументов
                        }

                        /*--------------------Отладка движения по Y-----------------------*/
                        if (historyOfYmove.getLast() == RobotStatusInDrive.StuckedBy_Y) {
                            historyOfYmove.removeLast();
                            robot.drivetrain.driveTrainStatus = RobotModuleStatus.Stucked;

                            if (historyOfYmove.getLast() == RobotStatusInDrive.MovingInOtherSideBy_Y) {
                                robot.drivetrain.motorsYdirection = MotorsStatus.Reversed;
                            }
                            if (historyOfYmove.getLast() == RobotStatusInDrive.MovingBy_Y) {
                                robot.drivetrain.motorsYdirection = MotorsStatus.Normal;
                            }

                            lastPos.setY(lastArgs.position.getY());
                        } else if (historyOfYmove.getLast() == RobotStatusInDrive.MovingInOtherSideBy_Y) {
                            robot.drivetrain.driveTrainStatus = RobotModuleStatus.Stucked;
                            lastPos.setY(lastArgs.position.getY());
                        } else {
                            //Остаётся статус Moving значит всё нормально
                            lastPos.setY(driveTask.args.position.getY());//Ничего не трогаем из прошлых аргументов
                        }

                        /*--------------------Отладка движения при повороте-----------------------*/
                        if (historyOfRotate.getLast() == RobotStatusInDrive.StuckedBy_Rotate) {
                            robot.drivetrain.driveTrainStatus = RobotModuleStatus.Stucked;
                            lastPos.setHeading(lastArgs.position.getHeading());
                        } else {
                            //Остаётся статус Moving значит всё нормально
                            lastPos.setHeading(driveTask.args.position.getHeading());//Ничего не трогаем из прошлых аргументов
                        }

                        newArgs = new StandartArgs.driveArgs(lastPos, lastArgs.max_linear_speed);

                        //Бросаем на выполнение прошлые аргументы, чтобы вернуться на прошлую позицию
                        if (robot.drivetrain.driveTrainStatus == RobotModuleStatus.Stucked){
                            robot.drivetrain.driveTrainStatus = RobotModuleStatus.Normal;

                            currentTask.startMode = OrdinaryTask.taskStartMode.HOTCAKE;
                            currentTask.state = States.FAILURE;

                            taskManager.addTaskToStack(
                                    new OrdinaryTask(currentTask.taskHandler,newArgs,
                                            OrdinaryTask.taskStartMode.HOTCAKE));//Исполняем ещё раз, если хоть что то надо исправить

                            taskManager.addTaskToStack( new OrdinaryTask(currentTask.taskHandler,currentTask.args,
                                    OrdinaryTask.taskStartMode.HOTCAKE));
                        }
                    } else if (currentTask instanceof TeleskopeTask) {

                        TeleskopeTask teleskopeTask = (TeleskopeTask) currentTask;

                        Deque<TeleskopeStatusInMoving> statusTeleskopeHist = teleskopeTask.teleskopeHandler.status()[0];

                        StandartArgs.teleskopeArgs newArgs;

                        if (statusTeleskopeHist.getLast() == TeleskopeStatusInMoving.StuckedBy_Downing) {
                            robot.teleSkope.motorsTeleskopeSt = RobotModuleStatus.Stucked;
                        } else if (statusTeleskopeHist.getLast() == TeleskopeStatusInMoving.StuckedBy_Upping) {
                            robot.teleSkope.motorsTeleskopeSt = RobotModuleStatus.Stucked;
                        }
                        newArgs =
                                new StandartArgs.teleskopeArgs(robot.teleSkope.getHeight(), robot.servosService.getHorizontal().getPosition(), 0.5);

                        if (robot.teleSkope.motorsTeleskopeSt == RobotModuleStatus.Stucked) {
                            robot.teleSkope.motorsTeleskopeSt = RobotModuleStatus.Normal;

                            currentTask.startMode = OrdinaryTask.taskStartMode.HOTCAKE;
                            currentTask.state = States.FAILURE;

                            taskManager.addTaskToStack(
                                    new OrdinaryTask(currentTask.taskHandler, newArgs,
                                            OrdinaryTask.taskStartMode.HOTCAKE));//Исполняем ещё раз, если хоть что то надо исправить

                            taskManager.addTaskToStack(currentTask);//Кидаем задачу ещё раз
                        }
                    }
                }
            }
        }
    }
}
