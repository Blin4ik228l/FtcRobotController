package org.firstinspires.ftc.teamcode.Modules.Examples.AutoParts;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Modules.Types.ExecutableModule;
import org.firstinspires.ftc.teamcode.Robot.RobotClass;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DrivetrainParts.TeamColorClass;

public class FindTagsLogic extends ExecutableModule {
    public FindTagsLogic(RobotClass.MecanumDrivetrain drivetrain, TeamColorClass teamColorClass, OpMode op) {
        super(op.telemetry);
        this.drivetrain = drivetrain;
        this.teamColorClass = teamColorClass;
    }
    public TeamColorClass teamColorClass;
    public RobotClass.MecanumDrivetrain drivetrain;
    public double minTurnAngle = Math.toRadians(10);

    // TODO \\
    @Override
    public void execute() {
        if(teamColorClass.color == TeamColorClass.Color.Red){
            if(teamColorClass.startPos == TeamColorClass.StartPos.Near_wall){
                if(Math.abs(drivetrain.odometryClass.encGlobalPosition2D.getHeading()) > 170) drivetrain.motors.setPower(0,0, -0.1);
                else drivetrain.motors.setPower(0,0,0);
            }else {
                if(Math.abs(drivetrain.odometryClass.encGlobalPosition2D.getHeading()) < minTurnAngle) drivetrain.motors.setPower(0,0, 0.1);
                else drivetrain.motors.setPower(0,0,0);
            }
        }else {
            if(teamColorClass.startPos == TeamColorClass.StartPos.Near_wall){
                if(Math.abs(drivetrain.odometryClass.encGlobalPosition2D.getHeading()) < minTurnAngle) drivetrain.motors.setPower(0,0, 0.1);
                else drivetrain.motors.setPower(0,0,0);
            }else {
                if(Math.abs(drivetrain.odometryClass.encGlobalPosition2D.getHeading()) < minTurnAngle) drivetrain.motors.setPower(0,0, -0.1);
                else drivetrain.motors.setPower(0,0,0);
            }
        }
    }
}
