package org.firstinspires.ftc.teamcode.Programms.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.BehaviorTree.Root;
import org.firstinspires.ftc.teamcode.BehaviorTree.States;
import org.firstinspires.ftc.teamcode.BehaviorTree.TaskNode;
import org.firstinspires.ftc.teamcode.Modules.RobotClass;
import org.firstinspires.ftc.teamcode.TaskAndArgs.Task;

@Autonomous(name = "BlueLeft", group = "Blue", preselectTeleOp = "BlueMeow")
public class AutoTest extends LinearOpMode {
    public RobotClass robot;
    public Thread parallelStream;
    public DriveTree driveTree;
    public TeleScopeTree teleScopeTree;


    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotClass(this);

        driveTree = new DriveTree(teleScopeTree);
        teleScopeTree = new TeleScopeTree(driveTree);

        parallelStream = new Thread(teleScopeTree);
        parallelStream.setDaemon(true);

        waitForStart();

        parallelStream.start();

        while (!isStopRequested() && opModeIsActive()){
            if(driveTree.mainRoot.root.nodeState != States.SUCCESS && driveTree.mainRoot.root.nodeState != States.RUNNING){
                driveTree.tickTree();
            }else{
                if (teleScopeTree.mainRoot.root.nodeState != States.SUCCESS && teleScopeTree.mainRoot.root.nodeState != States.RUNNING){
                    break;
                }
            }
        }
    }
   public class TeleScopeTree implements Runnable{
        public TeleScopeTree(DriveTree driveTree){
            mainRoot = new Root();

            this.driveTree = driveTree;
            loadProgram();

            mainRoot.root.stackNodes();
        }
        public Root mainRoot;

        private final DriveTree driveTree;
        private void loadProgram(){
           Task second = new Task(100, 0.5, robot.teleSkope, telemetry, 2);

           Task third = new Task("hook", 0.5, robot.teleSkope, telemetry, 3);
           Task fourth = new Task("flip", 0.5, robot.teleSkope, telemetry, 3);
           Task fifth = new Task("horizontal", 0.5, robot.teleSkope, telemetry, 4);

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
        @Override
        public void run() {
            tickTree();
        }

        public void tickTree(){
            pauseDriveTree();

            if (getQueuePlaceFromDriveTreeTask() == 1){//Если по параллельному потоку задача с первым по номеру,то она должна выполняться первой
                resumeDriveTree();
            }
            if(getSelfTaskQueuePlace() > getQueuePlaceFromDriveTreeTask()){//Если по параллельному потоку задача с меньшим номером ,то она должна выполняться первее
                resumeDriveTree();
            }

            if(getSelfTaskQueuePlace() == getQueuePlaceFromDriveTreeTask() && !teleScopeTree.mainRoot.root.stackToDo.peek().task.isDone){//Если по параллельному потоку задача с меньшим номером ,то она должна выполняться первее
                resumeDriveTree();
            }

            if(mainRoot.root.stackToDo.peek().task.isDone && driveTree.mainRoot.root.stackToDo.peek().task.isDone && getSelfTaskQueuePlace() - getQueuePlaceFromDriveTreeTask() == 0){//Данная строчка позволит продолжить выполнение после двух параллельных задач
                resumeDriveTree();
            }

            mainRoot.tick();
        }
    }
   public class DriveTree{
        public DriveTree(TeleScopeTree teleScopeTree){
            mainRoot = new Root();
            this.teleScopeTree = teleScopeTree;

            loadProgram();

            mainRoot.root.stackNodes();
        }
        public Root mainRoot;

        private final TeleScopeTree teleScopeTree;
        public void loadProgram(){
            Task first = new Task(100, 100, 0, 100, robot.driveTrain, telemetry, 1);

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
}
