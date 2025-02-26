package org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Pathes.Nodes.Main;

import org.firstinspires.ftc.teamcode.OpModes.Robot;
import org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Pathes.Conditions;
import org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Pathes.Nodes.Node;
import org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Pathes.PreSequences.SequenceRobotStuckOut;
import org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Pathes.States;
import org.firstinspires.ftc.teamcode.RobotCore.RobotStatus.RobotStatus;
import org.firstinspires.ftc.teamcode.RobotCore.RobotStatus.TeleskopeStatus;

public class Selector extends Node {
    //ControlNode (FallBack)
    //what to do next if a child returns FAILURE

    States selectorState;

    Robot robot;

    Conditions conditions;

    @Override
    public void tickMe() {

        while (selectorState != States.SUCCESS) {
            if (conditionMethod(conditions)) {
                if (conditions == Conditions.isRobotStucked) {
                    SequenceRobotStuckOut sequenceRobotStuckOut = new SequenceRobotStuckOut();

                    sequenceRobotStuckOut.tickMe();
                }
            }



        }

    }

    public void setCondition(Conditions conditions){
        this.conditions = conditions;
    }

    public Boolean conditionMethod(Conditions conditions) {
        if (conditions == Conditions.isRobotStucked) {
            if (robot.robotStatus == RobotStatus.Stucked) return true;
            else return false;
        } else if (conditions == Conditions.isTeleskopeStucked) {
            if (robot.teleskopeStatus == TeleskopeStatus.Stucked) return true;
            else return false;
        }else return null;
    }

    public void whatShouildItDo(Node...nodes){

    }
}
