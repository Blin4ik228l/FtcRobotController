package org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Node.OrdinalNodes.DecoratorNode.Decorators;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Node.Node;
import org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Node.OrdinalNodes.DecoratorNode.Decorator;

public class Delay extends Decorator {
    //Программа будет обращаться к следующему узлу через какое - то время, каждый раз
    public Delay(Node node, long time, LinearOpMode lin) {
        super(node);
        this.waitTime = time;
        this.lin = lin;
    }
    public long waitTime;
    public ElapsedTime delayTimer = new ElapsedTime();
    boolean stop;
    public LinearOpMode lin;

    @Override
    public void stopProgramm() {
        stop = true;
    }

    @Override
    public void programm() {
        if (delayTimer.seconds() < waitTime && !lin.isStopRequested() && lin.opModeIsActive()){
            return;
        }

        nodeToWork.tickMe();

        nodeState = nodeToWork.nodeState;
    }
}
