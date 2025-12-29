package org.firstinspires.ftc.teamcode.Trees;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Modules.Types.ExecutableModule;
import org.firstinspires.ftc.teamcode.Robot.RobotClass;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DriveTrain.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.TaskAndArgs.Task;
import org.firstinspires.ftc.teamcode.Trees.BehaviorTree.Root;
import org.firstinspires.ftc.teamcode.Trees.BehaviorTree.TaskNode;

public class DriveTree extends ExecutableModule {
    public DriveTree(MecanumDrivetrain drivetrain, AutomaticLogicTree automaticLogicTree, OpMode op){
        super(op.telemetry);
        mainRoot = new Root();

        this.drivetrain = drivetrain;
        this.teleScopeTree = automaticLogicTree;

        loadProgram();

        mainRoot.root.stackNodes();
    }
    public Root mainRoot;
    public MecanumDrivetrain drivetrain;
    private final AutomaticLogicTree teleScopeTree;
    public void loadProgram(){
        Task first = new Task(100, 100, 0, 100, drivetrain, telemetry, 1);

        mainRoot.add(new TaskNode(first));
    }
    public void pauseTeleSkopeTree(){
        teleScopeTree.mainRoot.root.stop = true;//Стопорим параллельный с этим поток, чтобы в дальнейшем их синхронизировать
    }

    public void resumeTeleSkopeTree(){
        teleScopeTree.mainRoot.root.stop = false;//Запускаем вновь дерево в параллельном потоке
    }

    public int getQueuePlaceFromTeleSkopeTreeTask(){
        return teleScopeTree.mainRoot.root.stackToDo.peek().task.queuePlace;
    }

    public int getSelfTaskQueuePlace(){
        return mainRoot.root.stackToDo.peek().task.queuePlace;
    }
    public void tickTree(){
        pauseTeleSkopeTree();

        if (getQueuePlaceFromTeleSkopeTreeTask() == 1){//Если по параллельному потоку задача с первым по номеру,то она должна выполняться первой
            resumeTeleSkopeTree();
        }

        if(getSelfTaskQueuePlace() > getQueuePlaceFromTeleSkopeTreeTask()){//Если по параллельному потоку задача с меньшим номером ,то она должна выполняться первее
            resumeTeleSkopeTree();
        }

        if(getSelfTaskQueuePlace() == getQueuePlaceFromTeleSkopeTreeTask() && !teleScopeTree.mainRoot.root.stackToDo.peek().task.isDone){//Если у обоих Task'ов одинаковое место в очереди, то они должны выполняться паралельно
            resumeTeleSkopeTree();
        }

        if(mainRoot.root.stackToDo.peek().task.isDone && teleScopeTree.mainRoot.root.stackToDo.peek().task.isDone && getSelfTaskQueuePlace() - getQueuePlaceFromTeleSkopeTreeTask() == 0){//Данная строчка позволит продолжить выполнение после двух параллельных задач
            resumeTeleSkopeTree();
        }

        mainRoot.tick();
    }
}
