package org.firstinspires.ftc.teamcode.Modules.Examples.AutoParts;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Modules.Types.ExecutableModule;
import org.firstinspires.ftc.teamcode.Robot.RobotClass;
import org.firstinspires.ftc.teamcode.Robot.TeamColor;

public class FindTagsLogic extends ExecutableModule {
    public FindTagsLogic(RobotClass.MecanumDrivetrain drivetrain, OpMode op) {
        super(op.telemetry);
        this.drivetrain = drivetrain;
    }
    public TeamColor teamColor;
    public RobotClass.MecanumDrivetrain drivetrain;
    public double minTurnAngle = Math.toRadians(10);

    // TODO \\
    @Override
    public void execute() {
        if(teamColor.color == TeamColor.Color.Red){
            if(teamColor.startPos == TeamColor.StartPos.Near_wall){
                if(Math.abs(drivetrain.exOdometry.encGlobalPosition2D.getHeading()) > 170) drivetrain.motors.setPower(0,0, -0.1);
                else drivetrain.motors.setPower(0,0,0);
            }else {
                if(Math.abs(drivetrain.exOdometry.encGlobalPosition2D.getHeading()) < minTurnAngle) drivetrain.motors.setPower(0,0, 0.1);
                else drivetrain.motors.setPower(0,0,0);
            }
        }else {
            if(teamColor.startPos == TeamColor.StartPos.Near_wall){
                if(Math.abs(drivetrain.exOdometry.encGlobalPosition2D.getHeading()) < minTurnAngle) drivetrain.motors.setPower(0,0, 0.1);
                else drivetrain.motors.setPower(0,0,0);
            }else {
                if(Math.abs(drivetrain.exOdometry.encGlobalPosition2D.getHeading()) < minTurnAngle) drivetrain.motors.setPower(0,0, -0.1);
                else drivetrain.motors.setPower(0,0,0);
            }
        }
    }
}
