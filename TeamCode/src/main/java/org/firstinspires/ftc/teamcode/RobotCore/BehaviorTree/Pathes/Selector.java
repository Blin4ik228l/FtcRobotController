package org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Pathes;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.OpModes.Robot;
import org.firstinspires.ftc.teamcode.RobotCore.RobotStatus.DriveTrainStatus;
import org.firstinspires.ftc.teamcode.RobotCore.RobotStatus.MotorsStatus;
import org.firstinspires.ftc.teamcode.RobotCore.RobotStatus.RobotStatusInDrive;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.StandartArgs;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.Tasks.OrdinaryTask;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.Tasks.VarietiesTasks.DriveTask;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.Tasks.VarietiesTasks.TeleskopeTask;
import org.firstinspires.ftc.teamcode.RobotCore.Utils.Position;

import java.util.Deque;
import java.util.Stack;

class Selector {
    private ElapsedTime runtime = new ElapsedTime();
    private Robot robot;
    //1.Дейстивие после выполнения вернёт результат
    //2.Обработчик результата
    //3.Конечный выбор

    public Selector(Robot robot){
        this.robot = robot;
    }

    public OrdinaryTask.States resultHandler(OrdinaryTask task){
        //Запускаем время с начала выполнения задачи
        runtime.seconds();
        if (isSuccessed(task.state)) {
            runtime.reset();
        }
        else{
            //Если время превысило n - ое кол - во секунд, тогда возвращаем статус FAILURE
            if (runtime.seconds() > 5){
                runtime.reset();
                return OrdinaryTask.States.FAILURE;
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
            if(task instanceof DriveTask){
                DriveTask driveTask = (DriveTask) task;//Текущие аргументы
                Deque<RobotStatusInDrive> historyOfXmove = driveTask.driveHandler.statusInDrive()[0];
                Deque<RobotStatusInDrive> historyOfYmove = driveTask.driveHandler.statusInDrive()[1];
                Deque<RobotStatusInDrive> historyOfRotate = driveTask.driveHandler.statusInDrive()[2];

                StandartArgs.driveStandartArgs lastArgs = null;
                Position lastPos = new Position();//Переменная для сохранения позиции из прошлой задачи

                Stack<OrdinaryTask> completedTasksInManager = robot.taskManager.getCompletedTasks();//Не трогаем целостность очереди из выполненых задач

                StandartArgs.driveStandartArgs newArgs;

                //Смотрим из очереди выполненых задач на прошлую выполненую задачу, чтобы например вернутся на прошлую позицию
                if(completedTasksInManager.peek().taskHandler == robot.driveToPosition) {
                    lastArgs = (StandartArgs.driveStandartArgs) completedTasksInManager.peek().args;
                }else{
                    //Ищем в очереди выполненых задач, задачу того же типа, чтобы попытаться вернутья в прошлою позицию
                    while(completedTasksInManager.peek().taskHandler != robot.driveToPosition){
                        completedTasksInManager.pop();

                        if(completedTasksInManager.peek().taskHandler == robot.driveToPosition){
                            lastArgs = (StandartArgs.driveStandartArgs) completedTasksInManager.peek().args;
                        }

                        if(robot.taskManager.getCompletedTasks().isEmpty()){
                            lastArgs = new StandartArgs.driveStandartArgs(robot.odometry.getStartGlobalPosition());
                            break;
                        }
                    }
                }

                //TODO: Дописать для статуса Moving
                /*--------------------Отладка движения по X-----------------------*/
                if(historyOfXmove.getLast() == RobotStatusInDrive.StuckedBy_X){
                    historyOfXmove.removeLast();
                    robot.drivetrain.driveTrainStatus = DriveTrainStatus.EmergStopped;

                    if(historyOfXmove.getLast() == RobotStatusInDrive.MovingInOtherSideBy_X){
                        robot.drivetrain.motorsXdirection = MotorsStatus.Reversed;
                        lastPos.setX(lastArgs.position.getX());
                    }
                    if(historyOfXmove.getLast() == RobotStatusInDrive.MovingBy_X){
                        robot.drivetrain.motorsXdirection = MotorsStatus.Normal;
                    }
                    lastPos.setX(lastArgs.position.getX());
                }else if(historyOfXmove.getLast() == RobotStatusInDrive.MovingInOtherSideBy_X){
                    robot.drivetrain.driveTrainStatus = DriveTrainStatus.EmergStopped;
                    lastPos.setX(lastArgs.position.getX());
                }else {
                    //Остаётся статус Moving значит всё нормально
                    lastPos.setX(driveTask.args.position.getX());//Ничего не трогаем из прошлых аргументов
                }


                /*--------------------Отладка движения по Y-----------------------*/
                if(historyOfYmove.getLast() == RobotStatusInDrive.StuckedBy_Y){
                    historyOfYmove.removeLast();
                    robot.drivetrain.driveTrainStatus = DriveTrainStatus.EmergStopped;

                    if(historyOfYmove.getLast() == RobotStatusInDrive.MovingInOtherSideBy_Y){
                        robot.drivetrain.motorsYdirection = MotorsStatus.Reversed;
                    }
                    if(historyOfYmove.getLast() == RobotStatusInDrive.MovingBy_Y){
                        robot.drivetrain.motorsYdirection = MotorsStatus.Normal;
                    }

                    lastPos.setY(lastArgs.position.getY());
                } else if (historyOfYmove.getLast() == RobotStatusInDrive.MovingInOtherSideBy_Y){
                    robot.drivetrain.driveTrainStatus = DriveTrainStatus.EmergStopped;
                    lastPos.setY(lastArgs.position.getY());
                }else {
                    //Остаётся статус Moving значит всё нормально
                    lastPos.setY(driveTask.args.position.getY());//Ничего не трогаем из прошлых аргументов
                }

                /*--------------------Отладка движения при повороте-----------------------*/
                if(historyOfRotate.getLast() == RobotStatusInDrive.StuckedBy_Rotate){
                    robot.drivetrain.driveTrainStatus = DriveTrainStatus.EmergStopped;
                    lastPos.setHeading(lastArgs.position.getHeading());
                }else {
                    //Остаётся статус Moving значит всё нормально
                    lastPos.setHeading(driveTask.args.position.getHeading());//Ничего не трогаем из прошлых аргументов
                }

                newArgs = new StandartArgs.driveStandartArgs(lastPos, lastArgs.max_linear_speed);

                //Бросаем на выполнение прошлые аргументы, чтобы вернуться на прошлую позицию
                if(robot.drivetrain.driveTrainStatus == DriveTrainStatus.EmergStopped)robot.drivetrain.driveTrainStatus = DriveTrainStatus.Normal;
                task.taskHandler.execute(robot.taskManager, newArgs);
            } else if (task instanceof TeleskopeTask) {
                TeleskopeTask teleskopeTask = (TeleskopeTask) task;


            }
        }
        return OrdinaryTask.States.RUNNING;
    }

    public boolean isSuccessed(OrdinaryTask.States state){
        return state == OrdinaryTask.States.SUCCESS;
    }
}
