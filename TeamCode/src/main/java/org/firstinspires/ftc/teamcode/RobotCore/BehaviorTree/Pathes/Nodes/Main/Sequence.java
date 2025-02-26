package org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Pathes.Nodes.Main;

import org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Pathes.Nodes.Node;
import org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Pathes.States;

import java.util.ArrayDeque;
import java.util.Deque;
import java.util.Iterator;
import java.util.Stack;

public class Sequence extends Node {
    //ControllNode
    private Selector selector;

    private final Deque<Node> childrens = new ArrayDeque<>();
    private final Stack<Node> stackChildrens = new Stack<>();

    public States sequenceState = States.WAITING;

    int succesCount = 0;


    @Override
    public void tickMe(){
        if(sequenceState != States.RUNNING && sequenceState != States.FAILURE) sequenceState = States.RUNNING;

        tickChildren();

        if(stackChildrens.peek().childrenState == States.SUCCESS){
            succesCount ++;
            stackChildrens.pop();
        }

        if(succesCount == childrens.size()) {
            sequenceState = States.SUCCESS;
        }

        if(stackChildrens.peek().childrenState == States.FAILURE){
            sequenceState = States.FAILURE;
        }
    }

    public void tickChildren(){
      stackChildrens.peek().tickMe();
    }


    public void addChildren(TaskNode children){
        childrens.addFirst(children);
    }

    public void addFallBack(Selector selector){
        childrens.addFirst(selector);
    }

    public void stackChildrens(){
        for (Iterator<Node> iterator = childrens.iterator(); iterator.hasNext();) {
            Node child = iterator.next();

            stackChildrens.push(child);
        }
    }


//    public void addAction(OrdinaryTask...tasks){
//        // tasks - это массив[]
//        runningTasks = tasks;
//        failureTasks = new OrdinaryTask[tasks.length];
//
//        actionHandler();
//    }
//
//    public OrdinaryTask[] getFailureTasks() {
//        return failureTasks;
//    }

//    public void actionHandler(){
//        for (int i = 0; i < runningTasks.length; i++) {
//            if(selector.resultHandler(runningTasks[i]) == OrdinaryTask.States.FAILURE){
//                failureTasks[i] = runningTasks[i];
//                failureTasks[i].startMode = OrdinaryTask.taskStartMode.HOTCAKE;
//            }
//        }
//    }


//    public void programm(){
//        for (int i = 0; i < runningTasks.length; i++) {
//            if(selector.isSuccessed(runningTasks[i].state)) return;
//        }
//    }
}
