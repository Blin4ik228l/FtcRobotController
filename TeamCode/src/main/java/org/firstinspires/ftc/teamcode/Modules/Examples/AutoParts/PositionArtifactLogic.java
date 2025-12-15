package org.firstinspires.ftc.teamcode.Modules.Examples.AutoParts;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Modules.Types.UpdatableModule;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DrivetrainParts.Odometry.OdometryClass;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DrivetrainParts.Odometry.Parts.MathUtils.Position2D;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DrivetrainParts.TeamColorClass;

public class PositionArtifactLogic extends UpdatableModule {

    public PositionArtifactLogic(OdometryClass odometryClass, TeamColorClass teamColorClass, OpMode op) {
        super(op.telemetry);

        this.odometryClass = odometryClass;
        this.teamColorClass = teamColorClass;
    }
    private final TeamColorClass teamColorClass;
    private final OdometryClass odometryClass;

    public LogicStates logicStates = LogicStates.Check_Pos;
    private MovementLogic movementLogic;

    private Position2D foundedPos = new Position2D();
    private Position2D posForSend = null;

    public Position2D getPosForSend() {
        return posForSend;
    }
    public void deleteArtifact(){
        teamColorClass.getClosestArtifacts()[minI][0] = 0;
    }

    int minI;
    int mx = 2, mn = -1;
    public enum LogicStates{
        Check_Pos,
        Find_nearest_art,
        Send_founded_pos
    }
    public enum MovementLogic{
        Up_to_down,
        Down_to_up
    }
    @Override
    public void update() {
        switch (logicStates){
            case Check_Pos:
                //Корды робота
                if(odometryClass.encGlobalPosition2D.getX() > 0){
                    minI = 8;
                    movementLogic = MovementLogic.Down_to_up;
                }else {
                    minI = 2;
                    movementLogic = MovementLogic.Up_to_down;
                }
                logicStates = LogicStates.Find_nearest_art;
                break;
            case Find_nearest_art:
                switch (movementLogic){
                    case Down_to_up:
                        if(minI == 0 && teamColorClass.getClosestArtifacts()[minI][0] == 0){
                            break;
                        }
                        if(teamColorClass.getClosestArtifacts()[minI][0] == 0){
                            minI --;
                            break;
                        }

                        break;
                    case Up_to_down:
                        if(teamColorClass.getClosestArtifacts()[minI][0] == 0){
                            minI --;
                            minI = Range.clip(minI, mn, mx);
                        }

                        if(minI == -1){
                            minI = 5;
                            mn = 2;
                            mx = 5;
                        } else if (minI == 2) {
                            minI = 8;
                            mn = 5;
                            mx = 8;
                        }else if(minI == 5){
                            break;
                        }
                        break;
                }

                foundedPos.setX(teamColorClass.getClosestArtifacts()[minI][1]);
                foundedPos.setY(teamColorClass.getClosestArtifacts()[minI][2]);
                foundedPos.setHeading(teamColorClass.getClosestArtifacts()[minI][4]);

                posForSend = foundedPos;
                logicStates = LogicStates.Send_founded_pos;
                break;

            case Send_founded_pos:
                if(teamColorClass.getClosestArtifacts()[minI][0] == 0){
                    logicStates = LogicStates.Check_Pos;
                }
                break;
        }

    }
}
