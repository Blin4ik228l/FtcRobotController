package org.firstinspires.ftc.teamcode.Modules.Examples.Players.PL0;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Modules.Examples.Players.Pl1.SemiAutoPlayerClass1;
import org.firstinspires.ftc.teamcode.Modules.Examples.Players.Pl2.AutoPlayerClass2;
import org.firstinspires.ftc.teamcode.Modules.Types.ExecutableModule;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DriveTrain.PositionRobotController;

public class MainSystem extends ExecutableModule {
    public SemiAutoPlayerClass1 semiAutoPlayerClass1;
    public AutoPlayerClass2 autoPlayerClass2;
    public MainSystem(SemiAutoPlayerClass1 semiAutoPlayerClass1, AutoPlayerClass2 autoPlayerClass2, OpMode op){
        super(op);
        this.semiAutoPlayerClass1 = semiAutoPlayerClass1;
        this.autoPlayerClass2 = autoPlayerClass2;

        AutoPlayerClass2.GeneralState generalState1 = null;
        AutoPlayerClass2.LoadState loadState = null;
        AutoPlayerClass2.FireState fireState = null;
        AutoPlayerClass2.AnotherStates anotherStates = null;
        PositionRobotController.AutoState autoState = null;
        PositionRobotController.GeneralState generalState2 = null;
    }
    public AutoPlayerClass2.GeneralState generalState1;
    public AutoPlayerClass2.LoadState loadState;
    public AutoPlayerClass2.FireState fireState;
    public AutoPlayerClass2.AnotherStates anotherStates;
    public PositionRobotController.AutoState autoState;
    public PositionRobotController.GeneralState generalState2;
    @Override
    public void execute() {
        if (semiAutoPlayerClass1.drivetrain.positionRobotController.autoState == PositionRobotController.AutoState.Check_readiness_for_start){
            if (autoPlayerClass2.generalState == AutoPlayerClass2.GeneralState.FireLogic && autoPlayerClass2.fireState == AutoPlayerClass2.FireState.Fire) {
                generalState1 = AutoPlayerClass2.GeneralState.FireLogic;
                fireState = AutoPlayerClass2.FireState.Fire;
                autoPlayerClass2.generalState = AutoPlayerClass2.GeneralState.Stop;
            }
        }else {
            if (autoPlayerClass2.generalState == AutoPlayerClass2.GeneralState.Stop) {
                if (generalState1 != null) autoPlayerClass2.generalState = generalState1;
                if (fireState != null) autoPlayerClass2.fireState = fireState;

                generalState1 = null;
                fireState = null;
            }
        }

        if (semiAutoPlayerClass1.drivetrain.positionRobotController.generalState == PositionRobotController.GeneralState.Execute_in_proccess
                && semiAutoPlayerClass1.drivetrain.positionRobotController.autoState == PositionRobotController.AutoState.Find_and_go_to_artifacts){
            if (autoPlayerClass2.generalState == AutoPlayerClass2.GeneralState.FireLogic && autoPlayerClass2.fireState == AutoPlayerClass2.FireState.Fire) {
                generalState2 = semiAutoPlayerClass1.drivetrain.positionRobotController.generalState;
                semiAutoPlayerClass1.drivetrain.positionRobotController.generalState = PositionRobotController.GeneralState.Stop;
            }
        }else {
            if (autoPlayerClass2.generalState == AutoPlayerClass2.GeneralState.LoadLogic) {
                if (generalState2 != null) semiAutoPlayerClass1.drivetrain.positionRobotController.generalState = generalState2;

                generalState2 = null;
            }
        }

        if (semiAutoPlayerClass1.drivetrain.positionRobotController.generalState == PositionRobotController.GeneralState.Execute_in_proccess
                && semiAutoPlayerClass1.drivetrain.positionRobotController.autoState == PositionRobotController.AutoState.Find_and_go_to_artifacts && autoPlayerClass2.collector.digitalCellsClass.isMaxed) {

            semiAutoPlayerClass1.drivetrain.positionRobotController.generalState = PositionRobotController.GeneralState.Get_pos;
            semiAutoPlayerClass1.drivetrain.positionRobotController.autoState = PositionRobotController.AutoState.Find_and_go_to_fire_pos;
        }
    }
}
