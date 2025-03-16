package org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Node.TaskNodes.TaskDecorator.TaskDecorators;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.OpModes.Robot;
import org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Node.Node;
import org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Node.OrdinalNodes.DecoratorNode.Decorator;
import org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Node.TaskNodes.TaskDecorator.TaskDecorator;
import org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Node.TaskNodes.TaskNode.TaskNode;
import org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.States;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.Tasks.OrdinaryTask;

public class RepeatUntilOnesDone extends TaskDecorator {
    public RepeatUntilOnesDone(Robot r, TaskNode node, LinearOpMode lin, Node...nodes) {
        super(r, lin);

        this.nodes = nodes;
        this.nodeToWork = node;
        this.lin = lin;

        this.task = node.task;

        countNodes = nodes.length;
    }
   public LinearOpMode lin;
   public Node[] nodes;
   public int countNodes;

   public int completedNodes = 0;
   public int i = 0;
    @Override
    public void programm() {
        if(nodeToWork.nodeState == States.RUNNING || nodeToWork.nodeState == States.TODO
                && nodes[i].nodeState == States.RUNNING || nodes[i].nodeState == States.TODO ) {
            nodeToWork.tickMe();
            nodes[i].tickMe();
        }


        if(nodes[i].nodeState == States.SUCCESS){
            completedNodes++;
        }

        if(completedNodes != countNodes && nodes[i].nodeState == States.SUCCESS){
            i++;
        }

        if(completedNodes == countNodes && nodeToWork.nodeState == States.SUCCESS){
            nodeState = States.SUCCESS;
            return;
        }

        if(nodeToWork.task.state == States.SUCCESS && countNodes != completedNodes){
            nodeToWork.task = new OrdinaryTask(task.taskHandler, task.args, task.startMode);
        }

        lin.telemetry.addData("nodeToWork.state", nodeToWork.nodeState);
        lin.telemetry.addData("nodes[i].nodeState", nodes[i].nodeState);
        lin.telemetry.update();
    }
}
