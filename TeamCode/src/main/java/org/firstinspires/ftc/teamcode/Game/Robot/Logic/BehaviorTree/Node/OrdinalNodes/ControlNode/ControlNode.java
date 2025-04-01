package org.firstinspires.ftc.teamcode.Game.Robot.Logic.BehaviorTree.Node.OrdinalNodes.ControlNode;

import org.firstinspires.ftc.teamcode.Game.Robot.Logic.BehaviorTree.Node.Node;

import java.util.ArrayDeque;
import java.util.Deque;
import java.util.Stack;

public abstract class ControlNode extends Node {
    public Stack<Node> stackToDo = new Stack();
    public Deque<Node> nodes = new ArrayDeque<>();

    public int completCount = 0;//для sequence'а
    public int failureCount = 0;//для selector'а

    public void addNode(Node node){
        nodes.addFirst(node);
    }

    public void stackNodes(){
        for (Node child : nodes) {
            stackToDo.push(child);
        }
    }

    public void tickNode(){
        stackToDo.peek().tickMe();
    }
}
