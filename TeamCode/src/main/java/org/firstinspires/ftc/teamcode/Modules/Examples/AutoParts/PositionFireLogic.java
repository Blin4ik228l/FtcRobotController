package org.firstinspires.ftc.teamcode.Modules.Examples.AutoParts;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Modules.Types.UpdatableModule;
import org.firstinspires.ftc.teamcode.Robot.RobotClass;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DrivetrainParts.Odometry.Parts.MathUtils.Position2D;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DrivetrainParts.Odometry.Parts.MathUtils.Vector2;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DrivetrainParts.TeamColorClass;

public class PositionFireLogic extends UpdatableModule {
    public PositionFireLogic(RobotClass.MecanumDrivetrain drivetrain, OpMode op) {
        super(op.telemetry);

        this.drivetrain = drivetrain;
        this.teamColorClass = drivetrain.teamColorClass;
    }
    public RobotClass.MecanumDrivetrain drivetrain;
    public TeamColorClass teamColorClass;

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
                    Vector2 deltaVector = new Vector2(teamColorClass.getFireZones()[i][0] - drivetrain.odometryClass.encGlobalPosition2D.getX(), teamColorClass.getFireZones()[i][1] - drivetrain.odometryClass.encGlobalPosition2D.getY());
                    if(deltaVector.length() < minRange){
                        minI = i;
                        minRange = deltaVector.length();
                        foundedPos = new Position2D(teamColorClass.getFireZones()[i][0], teamColorClass.getFireZones()[i][1], 0);
                    }
                }
                foundedPos.setHeading(drivetrain.positionRobotController.getDeltaAngle());

                sendedPos = foundedPos;

                logicStates = LogicStates.Send_pos;
                break;
            case Send_pos:
                Vector2 deltaVector = new Vector2(teamColorClass.getFireZones()[minI][0] - drivetrain.odometryClass.encGlobalPosition2D.getX(), teamColorClass.getFireZones()[minI][1] - drivetrain.odometryClass.encGlobalPosition2D.getY());
                if(deltaVector.length() < 1){
                    logicStates = LogicStates.Check_pos;
                }
                break;
        }

    }
}
