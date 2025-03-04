package org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Node.TaskNodes.TaskNode.TaskNodes;

import org.firstinspires.ftc.teamcode.OpModes.Robot;
import org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Node.TaskNodes.TaskNode.TaskNode;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.Tasks.OrdinaryTask;

public class ParallelActions extends TaskNode {
    public ParallelActions(Robot robot, TaskNode parallel1, TaskNode parallel2) {
        super(robot);
        parallel1.isProgrammDisabled = true;
        parallel2.isProgrammDisabled = true;

        parallel1.task.startMode = OrdinaryTask.taskStartMode.START_WITH_PREVIOUS;
        parallel2.task.startMode = OrdinaryTask.taskStartMode.START_WITH_PREVIOUS;

        this.parallel1 = parallel1;
        this.parallel2 = parallel2;
    }
    public TaskNode parallel1;
    public TaskNode parallel2;

    @Override
    public void programm() {
        if(!isAdded){
            robot.taskManager.addTask(parallel1.task);
            robot.taskManager.addTask(parallel2.task);
        }

        robot.taskManager.startDoing();
    }
}
