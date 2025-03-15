package org.firstinspires.ftc.teamcode.OpModes.Autonoms.Classes.Blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Consts.Consts;
import org.firstinspires.ftc.teamcode.Consts.ConstsTeleskope;
import org.firstinspires.ftc.teamcode.Consts.RewardsForActions;
import org.firstinspires.ftc.teamcode.OpModes.Autonoms.LinearOpModeModified;
import org.firstinspires.ftc.teamcode.OpModes.Robot;
import org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Node.OrdinalNodes.DecoratorNode.Decorators.Delay;
import org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Node.TaskNodes.TaskNode.TaskNodes.DriveTo;
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

//        startNode.add((new DriveTo(r, new StandartArgs.driveArgs(new Position(20, 30,135), 160))));
//        startNode.add(new DriveTo(r, new StandartArgs.driveArgs(new Position(40, 36,0), 160)));
//
//        startNode.add(new MoveHorizontTeleTo(r, 0.4));
//        startNode.add(new MoveHorizontTeleTo(r, -0.4));
////        startNode.add(new TakeInElementSeq(r));
//        startNode.add(new DriveTo(r, new StandartArgs.driveArgs(new Position(40, 38,-225), 160)));

        startNode.add(new Delay(
                new DriveTo(r, new StandartArgs.driveArgs(new Position(0, 0,90), 160)), 2000, this));


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
