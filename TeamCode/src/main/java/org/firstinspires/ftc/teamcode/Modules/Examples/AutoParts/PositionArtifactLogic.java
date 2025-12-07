package org.firstinspires.ftc.teamcode.Modules.Examples.AutoParts;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Modules.Types.UpdatableModule;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DrivetrainParts.Odometry.ExOdometry;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DrivetrainParts.Odometry.Parts.MathUtils.Position2D;
import org.firstinspires.ftc.teamcode.Robot.TeamColor;

public class PositionArtifactLogic extends UpdatableModule {

    public PositionArtifactLogic(ExOdometry exOdometry, TeamColor teamColor, OpMode op) {
        super(op.telemetry);

        this.exOdometry = exOdometry;
        this.teamColor = teamColor;
    }
    private final TeamColor teamColor;
    private final ExOdometry exOdometry;
    private LogicStates logicStates;
    private MovementLogic movementLogic;
    private Position2D foundedPos = new Position2D();
    private Position2D posForSend = null;

    public Position2D getPosForSend() {
        return posForSend;
    }
    public void deleteArtifact(){
        teamColor.getClosestArtifacts()[minI][0] = 0;
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
                if(exOdometry.encGlobalPosition2D.getX() > 0){
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
                        if(minI == 0 && teamColor.getClosestArtifacts()[minI][0] == 0){
                            break;
                        }
                        if(teamColor.getClosestArtifacts()[minI][0] == 0){
                            minI --;
                            break;
                        }

                        break;
                    case Up_to_down:
                        if(teamColor.getClosestArtifacts()[minI][0] == 0){
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

                foundedPos.setX(teamColor.getClosestArtifacts()[minI][1]);
                foundedPos.setY(teamColor.getClosestArtifacts()[minI][2]);
                foundedPos.setHeading(teamColor.getClosestArtifacts()[minI][4]);

                logicStates = LogicStates.Send_founded_pos;
                break;

            case Send_founded_pos:
                posForSend = foundedPos;
                if(teamColor.getClosestArtifacts()[minI][0] == 0){
                    logicStates = LogicStates.Check_Pos;
                }
                break;
        }

    }
}
