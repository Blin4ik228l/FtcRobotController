package org.firstinspires.ftc.teamcode.Trees;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Modules.Types.ExecutableModule;
import org.firstinspires.ftc.teamcode.Robot.RobotClass;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.Collector.Collector;
import org.firstinspires.ftc.teamcode.TaskAndArgs.Task;
import org.firstinspires.ftc.teamcode.Trees.BehaviorTree.Root;
import org.firstinspires.ftc.teamcode.Trees.BehaviorTree.TaskNode;

public class AutomaticLogicTree extends ExecutableModule {
    public AutomaticLogicTree(Collector collector, DriveTree driveTree, OpMode op){
        super(op.telemetry);

        mainRoot = new Root();

        this.driveTree = driveTree;
        loadProgram();

        mainRoot.root.stackNodes();
    }
    public Root mainRoot;
    public Collector collector;
    private final DriveTree driveTree;
    private void loadProgram(){
        Task second = new Task(100, 0.5, collector, telemetry, 2);

        Task third = new Task("hook", 0.5, collector, telemetry, 3);
        Task fourth = new Task("flip", 0.5, collector, telemetry, 3);
        Task fifth = new Task("horizontal", 0.5, collector, telemetry, 4);

        mainRoot.add(new TaskNode(second));
        mainRoot.add(new TaskNode(third));
    }
    public void pauseDriveTree(){
        driveTree.mainRoot.root.stop = true;//Стопорим параллельный с этим поток, чтобы в дальнейшем их синхронизировать
    }

    public void resumeDriveTree(){
        driveTree.mainRoot.root.stop = false;
    }

    public int getQueuePlaceFromDriveTreeTask(){
        return driveTree.mainRoot.root.stackToDo.peek().task.queuePlace;//Стопорим параллельный с этим поток, чтобы в дальнейшем их синхронизировать
    }

    public int getSelfTaskQueuePlace(){
        return mainRoot.root.stackToDo.peek().task.queuePlace;//Запускаем вновь дерево в параллельном потоке

    }

    public void tickTree(){
        pauseDriveTree();

        if (getQueuePlaceFromDriveTreeTask() == 1){//Если по параллельному потоку задача с первым по номеру,то она должна выполняться первой
            resumeDriveTree();
        }
        if(getSelfTaskQueuePlace() > getQueuePlaceFromDriveTreeTask()){//Если по параллельному потоку задача с меньшим номером ,то она должна выполняться первее
            resumeDriveTree();
        }

        if(getSelfTaskQueuePlace() == getQueuePlaceFromDriveTreeTask() && !driveTree.mainRoot.root.stackToDo.peek().task.isDone){//Если у обоих Task'ов одинаковое место в очереди, то они должны выполняться паралельно
            resumeDriveTree();
        }

        if(mainRoot.root.stackToDo.peek().task.isDone && driveTree.mainRoot.root.stackToDo.peek().task.isDone && getSelfTaskQueuePlace() - getQueuePlaceFromDriveTreeTask() == 0){//Данная строчка позволит продолжить выполнение после двух параллельных задач
            resumeDriveTree();
        }

        mainRoot.tick();
    }
}
