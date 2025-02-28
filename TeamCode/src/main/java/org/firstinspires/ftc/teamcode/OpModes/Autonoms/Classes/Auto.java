package org.firstinspires.ftc.teamcode.OpModes.Autonoms.Classes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.OpModes.Autonoms.LinearOpModeModified;
import org.firstinspires.ftc.teamcode.OpModes.Robot;
import org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Node.OrdinalNodes.ControlNode.ControlNodes.SelectorNode.Selector;
import org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Node.OrdinalNodes.ControlNode.ControlNodes.SequenceNode.Sequence;
import org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Node.OrdinalNodes.DecoratorNode.Decorators.Delay;
import org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Node.OrdinalNodes.DecoratorNode.Decorators.ForceSuccess;
import org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Node.OrdinalNodes.DecoratorNode.Decorators.Inverter;
import org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Node.OrdinalNodes.DecoratorNode.Decorators.Repeat;
import org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Node.TaskNodes.TaskNode.TaskNodes.DriveTo;
import org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Node.TaskNodes.TaskNode.TaskNodes.MoveHorizontTeleTo;
import org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Node.TaskNodes.TaskSequence.TaskSequences.TakeInElementSeq;
import org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Root;
import org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.States;
import org.firstinspires.ftc.teamcode.RobotCore.RobotUtils.RobotAlliance;
import org.firstinspires.ftc.teamcode.RobotCore.RobotUtils.RobotMode;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.StandartArgs;
import org.firstinspires.ftc.teamcode.RobotCore.Utils.Position;

@Disabled
public class Auto extends LinearOpModeModified {
    @Override
    public void runOpMode() throws InterruptedException {
        //Примерный вид программы с использованием BehaviorTree
        robot = new Robot(RobotMode.AUTO, RobotAlliance.RED, this, new Position(0,0,0));

        startNode = new Root(robot);

        startNode.add(new DriveTo(new StandartArgs.driveArgs(new Position())));//Добавляем узел DriveTo

        startNode.add(new MoveHorizontTeleTo(0.5));//Добавляем узел MoveHorizontal

        startNode.add(new DriveTo(new StandartArgs.driveArgs(new Position())));

        startNode.add(new Repeat(new TakeInElementSeq(), 5));

        startNode.add(new Delay(new DriveTo(new StandartArgs.driveArgs(new Position())), 1.5));

        //Или такой вид
        Sequence firstPart = new Sequence();

        firstPart.addNode(new DriveTo(new StandartArgs.driveArgs(new Position())));
        firstPart.addNode(new MoveHorizontTeleTo(0.5));

        Selector secondPart = new Selector();

        secondPart.addNode(new DriveTo(new StandartArgs.driveArgs(new Position())));
        secondPart.addNode(new Repeat(new TakeInElementSeq(), 5));

        Sequence endPart = new Sequence();

        endPart.addNode(new Delay(new DriveTo(new StandartArgs.driveArgs(new Position())), 1.5));

        startNode.add(new ForceSuccess(firstPart));
        startNode.add(secondPart);
        startNode.add(new Inverter(endPart));

        waitForStart();

        while(!isStopRequested() && opModeIsActive()){
            if(startNode.root.nodeState == States.SUCCESS)break;
            else if(startNode.root.nodeState == States.FAILURE) break;
            else startNode.tick();
        }
    }
}
