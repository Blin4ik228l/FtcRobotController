package org.firstinspires.ftc.teamcode.OpModes.Autonoms.Classes.Blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Consts.Consts;
import org.firstinspires.ftc.teamcode.Consts.ConstsTeleskope;
import org.firstinspires.ftc.teamcode.Consts.RewardsForActions;
import org.firstinspires.ftc.teamcode.OpModes.Autonoms.LinearOpModeModified;
import org.firstinspires.ftc.teamcode.OpModes.Robot;
import org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Node.TaskNodes.TaskNode.TaskNodes.DriveTo;
import org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Node.TaskNodes.TaskNode.TaskNodes.Teleskope;
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

        startNode.add((new DriveTo(r, new StandartArgs.driveArgs(new Position(100, 0,0), 150))));
//
        startNode.add((new DriveTo(r, new StandartArgs.driveArgs(new Position(0, 0,0)))));
//        startNode.add((new DriveTo(r, new StandartArgs.driveArgs(new Position(20, 60, 135)))));
//        startNode.add((new Teleskope(r, new StandartArgs.teleskopeArgs(20, 0, 0.7))));

//        startNode.add((new DriveTo(r, new StandartArgs.driveArgs(new Position(20, 60,0)))));

//        startNode.add((new DriveTo(r, new StandartArgs.driveArgs(new Position(20, 60, 135)))));


        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            if(startNode.root.nodeState == States.SUCCESS)break;
            else if(startNode.root.nodeState == States.FAILURE) break;
            else startNode.tick();
        }

        r.autoInterrupt();
    }
}
