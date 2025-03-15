package org.firstinspires.ftc.teamcode.OpModes.Autonoms.Classes.Blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Consts.Consts;
import org.firstinspires.ftc.teamcode.Consts.ConstsTeleskope;
import org.firstinspires.ftc.teamcode.Consts.RewardsForActions;
import org.firstinspires.ftc.teamcode.OpModes.Autonoms.LinearOpModeModified;
import org.firstinspires.ftc.teamcode.OpModes.Robot;
import org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Node.OrdinalNodes.DecoratorNode.Decorators.Delay;
import org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Node.TaskNodes.TaskNode.TaskNodes.DriveTo;
import org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Node.TaskNodes.TaskNode.TaskNodes.MoveHorizontTeleTo;
import org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Node.TaskNodes.TaskSequence.TaskSequences.PrepareToElement;
import org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Node.TaskNodes.TaskSequence.TaskSequences.TakeInElementSeq;
import org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Node.TaskNodes.TaskSequence.TaskSequences.TakeOutElementSeq;
import org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Root;
import org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.States;
import org.firstinspires.ftc.teamcode.RobotCore.RobotUtils.RobotAlliance;
import org.firstinspires.ftc.teamcode.RobotCore.RobotUtils.RobotMode;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.StandartArgs;
import org.firstinspires.ftc.teamcode.RobotCore.Utils.Position;

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
        startNode.add(new MoveHorizontTeleTo(r, 0.08, this));
        startNode.add(new TakeOutElementSeq(r, this));
        startNode.add(new MoveHorizontTeleTo(r, -0.08, this));

        //Взяли sample2
        startNode.add(new DriveTo(r, new StandartArgs.driveArgs(new Position(40, 24,0), 160), this));
        startNode.add(new PrepareToElement(r, this));
        startNode.add(new MoveHorizontTeleTo(r, 0.18, this));
        startNode.add(new TakeInElementSeq(r, this));
        startNode.add(new MoveHorizontTeleTo(r, -0.18, this));

        //Выгрузили sample2
        startNode.add(new DriveTo(r, new StandartArgs.driveArgs(new Position(22, 38,135), 160), this));
        startNode.add(new MoveHorizontTeleTo(r, 0.08, this));
        startNode.add(new TakeOutElementSeq(r, this));
        startNode.add(new MoveHorizontTeleTo(r, -0.08, this));
        startNode.add(new PrepareToElement(r, this));

        //Взяли sample3
        startNode.add(new DriveTo(r, new StandartArgs.driveArgs(new Position(40, 24,25), 160), this));
        startNode.add(new MoveHorizontTeleTo(r, 0.22, this));
        startNode.add(new TakeInElementSeq(r, this));
        startNode.add(new MoveHorizontTeleTo(r, -0.22, this));

        //Выгрузили sample3
        startNode.add(new DriveTo(r, new StandartArgs.driveArgs(new Position(22, 38,135), 160), this));
        startNode.add(new MoveHorizontTeleTo(r, 0.08, this));
        startNode.add(new TakeOutElementSeq(r, this));
        startNode.add(new MoveHorizontTeleTo(r, -0.08, this));
        startNode.add(new PrepareToElement(r, this));

        //Взяли sample4
        startNode.add(new DriveTo(r, new StandartArgs.driveArgs(new Position(60, 40,42), 160), this));
        startNode.add(new MoveHorizontTeleTo(r, 0.14, this));
        startNode.add(new TakeInElementSeq(r, this));
        startNode.add(new MoveHorizontTeleTo(r, -0.14, this));

        //Выгрузили sample4
        startNode.add(new DriveTo(r, new StandartArgs.driveArgs(new Position(22, 38,135), 160), this));
        startNode.add(new MoveHorizontTeleTo(r, 0.08, this));
        startNode.add(new TakeOutElementSeq(r, this));
        startNode.add(new MoveHorizontTeleTo(r, -0.08, this));

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            if(startNode.root.nodeState == States.SUCCESS){
                startNode.root.stopProgramm();
                break;
            }
            else if(startNode.root.nodeState == States.FAILURE) {
                startNode.root.stopProgramm();
                break;
            }
            else startNode.tick();
        }

        r.autoInterrupt();
    }
}
