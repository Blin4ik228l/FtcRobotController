package org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Node.OrdinalNodes.DecoratorNode.Decorators;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Node.OrdinalNodes.DecoratorNode.Decorator;
import org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Node.Node;

public class Delay extends Decorator {
    //Программа будет обращаться к следующему узлу через какое - то время, каждый раз
    public Delay(Node node, double time) {
        super(node);
        this.waitTime = time;
    }
    public double waitTime;
    public ElapsedTime delayTimer = new ElapsedTime();
    boolean stop;


    @Override
    public void stopProgramm() {
        stop = true;
    }

    @Override
    public void programm() {
        while (delayTimer.seconds() != waitTime && !stop){
            //waiting...
        }
        delayTimer.reset();
        nodeToWork.tickMe();

        nodeState = nodeToWork.nodeState;
    }
}
