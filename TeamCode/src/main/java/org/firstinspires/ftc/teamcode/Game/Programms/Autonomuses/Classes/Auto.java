package org.firstinspires.ftc.teamcode.Game.Programms.Autonomuses.Classes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Game.Programms.Autonomuses.LinearOpModeModified;

@Disabled
public class Auto extends LinearOpModeModified {
    @Override
    public void runOpMode() throws InterruptedException {
        //Примерный вид программы с использованием BehaviorTree
//        robot = new Robot(RobotMode.AUTO, RobotAlliance.RED, this, new Position(0,0,0));
//
//        startNode = new Root(robot);
//
//        startNode.add(new DriveTo(new StandartArgs.driveArgs(new Position())));//Добавляем узел DriveTo
//
//        startNode.add(new MoveHorizontTeleTo(0.5));//Добавляем узел MoveHorizontal
//
//        startNode.add(new DriveTo(new StandartArgs.driveArgs(new Position())));
//
//        startNode.add(new Repeat(new TakeInElementSeq(), 5));
//
//        startNode.add(new Delay(new DriveTo(new StandartArgs.driveArgs(new Position())), 1.5));
//
//        //Или такой вид
//        Sequence firstPart = new Sequence();
//
//        firstPart.addNode(new DriveTo(new StandartArgs.driveArgs(new Position())));
//        firstPart.addNode(new MoveHorizontTeleTo(0.5));
//
//        Selector secondPart = new Selector();
//
//        secondPart.addNode(new DriveTo(new StandartArgs.driveArgs(new Position())));
//        secondPart.addNode(new Repeat(new TakeInElementSeq(), 5));
//
//        Sequence endPart = new Sequence();
//
//        endPart.addNode(new Delay(new DriveTo(new StandartArgs.driveArgs(new Position())), 1.5));
//
//        startNode.add(new ForceSuccess(firstPart));
//        startNode.add(secondPart);
//        startNode.add(new Inverter(endPart));
//
//        waitForStart();
//
//        while(!isStopRequested() && opModeIsActive()){
//            if(startNode.root.nodeState == States.SUCCESS)break;
//            else if(startNode.root.nodeState == States.FAILURE) break;
//            else startNode.tick();
//        }
    }
}
