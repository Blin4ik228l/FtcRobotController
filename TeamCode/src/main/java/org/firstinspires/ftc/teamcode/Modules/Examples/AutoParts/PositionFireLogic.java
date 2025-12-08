package org.firstinspires.ftc.teamcode.Modules.Examples.AutoParts;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Modules.Types.UpdatableModule;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DrivetrainParts.Odometry.ExOdometry;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DrivetrainParts.Odometry.Parts.MathUtils.Position2D;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DrivetrainParts.Odometry.Parts.MathUtils.Vector2;
import org.firstinspires.ftc.teamcode.Robot.TeamColor;

public class PositionFireLogic extends UpdatableModule {
    public PositionFireLogic(ExOdometry exOdometry, TeamColor teamColor, OpMode op) {
        super(op.telemetry);

        this.exOdometry = exOdometry;
        this.teamColor = teamColor;
    }
    public ExOdometry exOdometry;
    public TeamColor teamColor;

    public LogicStates logicStates = LogicStates.Check_pos;

    private Position2D foundedPos = new Position2D();
    private Position2D sendedPos = null;
    public Position2D getSendedPos(){
        return sendedPos;
    }
    int minI;
    public enum LogicStates{
        Check_pos,
        Send_pos,
    }
    @Override
    public void update() {
        switch (logicStates){
            case Check_pos:
                double minRange = 355;
                minI = 0;
                for (int i = 0; i < 2; i++) {
                    Vector2 deltaVector = new Vector2(teamColor.getFireZones()[i][0] - exOdometry.encGlobalPosition2D.getX(), teamColor.getFireZones()[i][1] - exOdometry.encGlobalPosition2D.getY());
                    if(deltaVector.length() < minRange){
                        minI = i;
                        minRange = deltaVector.length();
                        foundedPos = new Position2D(teamColor.getFireZones()[i][0], teamColor.getFireZones()[i][1], 0);
                    }
                }
                foundedPos.setHeading(exOdometry.getDeltaAngle());

                sendedPos = foundedPos;

                logicStates = LogicStates.Send_pos;
                break;
            case Send_pos:
                Vector2 deltaVector = new Vector2(teamColor.getFireZones()[minI][0] - exOdometry.encGlobalPosition2D.getX(), teamColor.getFireZones()[minI][1] - exOdometry.encGlobalPosition2D.getY());
                if(deltaVector.length() < 1){
                    logicStates = LogicStates.Check_pos;
                }
                break;
        }

    }
}
