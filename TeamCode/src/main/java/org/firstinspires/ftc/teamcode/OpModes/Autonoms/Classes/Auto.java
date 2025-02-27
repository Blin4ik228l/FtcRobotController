package org.firstinspires.ftc.teamcode.OpModes.Autonoms.Classes;

import org.firstinspires.ftc.teamcode.OpModes.Autonoms.LinearOpModeModified;
import org.firstinspires.ftc.teamcode.OpModes.Robot;
import org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Pathes.Nodes.Main.Root;
import org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Pathes.TaskNodes.ActionNodes.DriveTo;
import org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Pathes.TaskNodes.ActionNodes.MoveHorizontTeleTo;
import org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.States;
import org.firstinspires.ftc.teamcode.RobotCore.RobotUtils.RobotAlliance;
import org.firstinspires.ftc.teamcode.RobotCore.RobotUtils.RobotMode;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.StandartArgs;
import org.firstinspires.ftc.teamcode.RobotCore.Utils.Position;


public class Auto extends LinearOpModeModified {
    @Override
    public void runOpMode() throws InterruptedException {
        //Примерный вид программы с использованием BehaviorTree
        robot = new Robot(RobotMode.AUTO, RobotAlliance.RED, this, new Position(0,0,0));

        root = new Root(robot);

        root.add(new DriveTo(new StandartArgs.driveArgs(new Position())));//Добавляем ветвь DriveTo

        root.add(new MoveHorizontTeleTo(0.5));//Добавляем ветвь MoveHorizontal

        root.add(new DriveTo(new StandartArgs.driveArgs(new Position())));

        waitForStart();

        while(!isStopRequested() && opModeIsActive()){
            if(root.nodeState == States.SUCCESS)break;
            else if(root.nodeState == States.FAILURE) break;
            else root.tickMe();
        }
    }
}
