package org.firstinspires.ftc.teamcode.Game.Programms.Autonomuses.Classes.Blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Game.Programms.Autonomuses.LinearOpModeModified;
import org.firstinspires.ftc.teamcode.Utils.Consts.Consts;
import org.firstinspires.ftc.teamcode.Utils.Consts.ConstsTeleskope;
import org.firstinspires.ftc.teamcode.Utils.Consts.RewardsForActions;
import org.firstinspires.ftc.teamcode.Game.Programms.Robot;
import org.firstinspires.ftc.teamcode.Game.Robot.Logic.BehaviorTree.Node.TaskNodes.TaskNode.TaskNodes.DriveTo;
import org.firstinspires.ftc.teamcode.Game.Robot.Logic.BehaviorTree.Node.TaskNodes.TaskNode.TaskNodes.MoveHorizontTeleTo;
import org.firstinspires.ftc.teamcode.Game.Robot.Logic.BehaviorTree.Node.TaskNodes.TaskNode.TaskNodes.MoveVerticalTeleTo;
import org.firstinspires.ftc.teamcode.Game.Robot.Logic.BehaviorTree.Node.TaskNodes.TaskSequence.TaskSequences.PrepareToElement;
import org.firstinspires.ftc.teamcode.Game.Robot.Logic.BehaviorTree.Node.TaskNodes.TaskSequence.TaskSequences.TakeInElementSeq;
import org.firstinspires.ftc.teamcode.Game.Robot.Logic.BehaviorTree.Node.TaskNodes.TaskSequence.TaskSequences.TakeOutElementSeq;
import org.firstinspires.ftc.teamcode.Game.Robot.Logic.BehaviorTree.Root;
import org.firstinspires.ftc.teamcode.Game.Robot.Logic.BehaviorTree.States;
import org.firstinspires.ftc.teamcode.Game.Robot.Logic.RobotUtils.RobotAlliance;
import org.firstinspires.ftc.teamcode.Game.Robot.Logic.RobotUtils.RobotMode;
import org.firstinspires.ftc.teamcode.Game.Robot.Logic.TaskUtils.StandartArgs;
import org.firstinspires.ftc.teamcode.Game.Robot.Logic.Utils.Position;

@Autonomous(name = "BlueLeft", group = "Blue", preselectTeleOp = "BlueMeow")
public class AutoBlueLeft extends LinearOpModeModified implements ConstsTeleskope, Consts, RewardsForActions {

    @Override
    public void runOpMode() throws InterruptedException {
        r = new Robot(RobotMode.AUTO, RobotAlliance.BLUE, this, new Position(0,0,0));
        r.init();
        r.robotStatusHandler.start();

        startNode = new Root();

        //Выгрузили sample1
        startNode.add(new PrepareToElement(r, this));
        startNode.add(new DriveTo(r, new StandartArgs.driveArgs(new Position(22, 38,135), 160), this));

        startNode.add(new MoveVerticalTeleTo(r, 96, this));
        startNode.add(new MoveHorizontTeleTo(r, 0.16, this));
        startNode.add(new TakeOutElementSeq(r, this));
        startNode.add(new PrepareToElement(r, this));
        startNode.add(new MoveHorizontTeleTo(r, -0.16, this));

        startNode.add(new MoveVerticalTeleTo(r, 1, this));
        startNode.add(new DriveTo(r, new StandartArgs.driveArgs(new Position(40, 23.5,0), 160), this));

        //Взяли sample2
        startNode.add(new MoveHorizontTeleTo(r, 0.18, this));
        startNode.add(new TakeInElementSeq(r, this));
        startNode.add(new MoveHorizontTeleTo(r, -0.18, this));

        //Выгрузили sample2
        startNode.add(new DriveTo(r, new StandartArgs.driveArgs(new Position(22, 38,135), 160), this));
        startNode.add(new MoveVerticalTeleTo(r, 96, this));
        startNode.add(new MoveHorizontTeleTo(r, 0.16, this));
        startNode.add(new TakeOutElementSeq(r, this));
        startNode.add(new PrepareToElement(r, this));
        startNode.add(new MoveHorizontTeleTo(r, -0.16, this));
        startNode.add(new MoveVerticalTeleTo(r, 1, this));

//        //Взяли sample3
        startNode.add(new DriveTo(r, new StandartArgs.driveArgs(new Position(40, 23.5,25), 160), this));
        startNode.add(new MoveHorizontTeleTo(r, 0.24, this));
        startNode.add(new TakeInElementSeq(r, this));
        startNode.add(new MoveHorizontTeleTo(r, -0.24, this));

//        //Выгрузили sample3
        startNode.add(new DriveTo(r, new StandartArgs.driveArgs(new Position(22, 38,135), 160), this));
        startNode.add(new MoveVerticalTeleTo(r, 96, this));
        startNode.add(new MoveHorizontTeleTo(r, 0.16, this));
        startNode.add(new TakeOutElementSeq(r, this));
        startNode.add(new PrepareToElement(r, this));
        startNode.add(new MoveHorizontTeleTo(r, -0.16, this));
        startNode.add(new MoveVerticalTeleTo(r, 1, this));

//        //Взяли sample4
        startNode.add(new DriveTo(r, new StandartArgs.driveArgs(new Position(59.5, 40,50), 160), this));
        startNode.add(new MoveHorizontTeleTo(r, 0.18, this));
        startNode.add(new TakeInElementSeq(r, this));
        startNode.add(new MoveHorizontTeleTo(r, -0.18, this));

//        //Выгрузили sample4
        startNode.add(new DriveTo(r, new StandartArgs.driveArgs(new Position(22, 38,135), 160), this));
        startNode.add(new MoveVerticalTeleTo(r, 96, this));
        startNode.add(new MoveHorizontTeleTo(r, 0.16, this));
        startNode.add(new TakeOutElementSeq(r, this));
        startNode.add(new PrepareToElement(r, this));

        startNode.add(new MoveHorizontTeleTo(r, -0.16, this));
        startNode.add(new MoveVerticalTeleTo(r, 1, this));

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            if(startNode.root.nodeState == States.SUCCESS){
                break;
            }
            else if(startNode.root.nodeState == States.FAILURE) {
                break;
            }
            else startNode.tick();
        }

        r.autoInterrupt();
    }
}