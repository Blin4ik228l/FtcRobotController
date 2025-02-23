package org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Pathes;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.OpModes.Robot;
import org.firstinspires.ftc.teamcode.RobotCore.RobotStatus.MotorsStatus;
import org.firstinspires.ftc.teamcode.RobotCore.RobotStatus.RobotStatusInDrive;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.StandartArgs;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.Task;
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

    public Task.States resultHandler(Task task){
        //Запускаем время с начала выполнения задачи
        runtime.seconds();
        if (isSuccessed(task.state)) {
            runtime.reset();
        }
        else{
            //Если время превысило n - ое кол - во секунд, тогда возвращаем статус FAILURE
            if (runtime.seconds() > 5){
                runtime.reset();
                return Task.States.FAILURE;
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
            if(task.taskHandler == robot.driveToPosition){
                Deque<RobotStatusInDrive> historyOfXmove = task.taskHandler.statusInDrive()[0];
                Deque<RobotStatusInDrive> historyOfYmove = task.taskHandler.statusInDrive()[1];
                Deque<RobotStatusInDrive> historyOfRotate = task.taskHandler.statusInDrive()[2];

                StandartArgs.driveStandartArgs lastArgs = null;
                Position lastPos = new Position();

                Stack<Task> completedTasksInManager = robot.taskManager.getCompletedTasks();//Не трогаем целостность очереди из выполненых задач

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

                if(historyOfXmove.getLast() == RobotStatusInDrive.StuckedBy_X){
                    historyOfXmove.removeLast();
                    if(historyOfXmove.getLast() == RobotStatusInDrive.MovingInOtherSideBy_X){
                        lastPos.setX(lastArgs.position.getX());
                        robot.drivetrain.motorsXdirection = MotorsStatus.Reversed;
                    }
                    if(historyOfXmove.getLast() == RobotStatusInDrive.MovingBy_X){
                        lastPos.setX(lastArgs.position.getX());
                        robot.drivetrain.motorsXdirection = MotorsStatus.Normal;
                    }
                }

                //TODO: Дописать для статуса Moving
                if(historyOfYmove.getLast() == RobotStatusInDrive.StuckedBy_Y){
                    historyOfYmove.removeLast();
                    if(historyOfYmove.getLast() == RobotStatusInDrive.MovingInOtherSideBy_Y){
                        lastPos.setY(lastArgs.position.getY());
                        robot.drivetrain.motorsYdirection = MotorsStatus.Reversed;
                    }
                    if(historyOfYmove.getLast() == RobotStatusInDrive.MovingInOtherSideBy_Y){
                        lastPos.setY(lastArgs.position.getY());
                        robot.drivetrain.motorsYdirection = MotorsStatus.Normal;
                    }
                }
                if(historyOfRotate.getLast() == RobotStatusInDrive.StuckedBy_Rotate){
                    lastPos.setHeading(lastArgs.position.getHeading());
                }

                newArgs = new StandartArgs.driveStandartArgs(lastPos, lastArgs.max_linear_speed);

                //Бросаем на выполнение прошлые аргументы, чтобы вернуться на прошлую позицию
                task.taskHandler.execute(robot.taskManager, newArgs);
            }
        }
        return Task.States.RUNNING;
    }

    public boolean isSuccessed(Task.States state){
        return state == Task.States.SUCCESS;
    }
}
