package org.firstinspires.ftc.teamcode.OpModes.Autonoms.Classes.Blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Consts.Consts;
import org.firstinspires.ftc.teamcode.Consts.ConstsTeleskope;
import org.firstinspires.ftc.teamcode.Consts.RewardsForActions;
import org.firstinspires.ftc.teamcode.OpModes.Autonoms.LinearOpModeModified;
import org.firstinspires.ftc.teamcode.OpModes.Robot;
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
        this.robot = new Robot(RobotMode.AUTO,RobotAlliance.BLUE, this, new Position(0,0,0));
        this.robot.init();

        startNode = new Root(this.robot);

        startNode.add(new DriveTo(new StandartArgs.driveArgs(new Position(100, 0,0)), robot));

        startNode.add(new DriveTo(new StandartArgs.driveArgs(new Position(0, 0,0)), robot));

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            if(startNode.root.nodeState == States.SUCCESS)break;
            else if(startNode.root.nodeState == States.FAILURE) break;
            else startNode.tick();
        }

    }
}
