package org.firstinspires.ftc.teamcode.Modules.Examples.AutoParts;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Modules.Types.ExecutableModule;
import org.firstinspires.ftc.teamcode.Robot.RobotClass;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DriveTrain.DrivetrainParts.TeamClass;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DriveTrain.MecanumDrivetrain;

public class FindTagsLogic extends ExecutableModule {
    public FindTagsLogic(MecanumDrivetrain drivetrain, TeamClass teamClass, OpMode op) {
        super(op.telemetry);
        this.drivetrain = drivetrain;
        this.teamClass = teamClass;
    }
    public TeamClass teamClass;
    public MecanumDrivetrain drivetrain;
    public double minTurnAngle = Math.toRadians(10);

    // TODO \\
    @Override
    public void execute() {
        if(teamClass.color == TeamClass.Color.Red){
            if(teamClass.startPos == TeamClass.StartPos.Near_wall){
                if(Math.abs(drivetrain.odometryClass.getEncGlobalPosition2D().getHeading()) > 170) drivetrain.motors.setPower(0,0, -0.1);
                else drivetrain.motors.setPower(0,0,0);
            }else {
                if(Math.abs(drivetrain.odometryClass.getEncGlobalPosition2D().getHeading()) < minTurnAngle) drivetrain.motors.setPower(0,0, 0.1);
                else drivetrain.motors.setPower(0,0,0);
            }
        }else {
            if(teamClass.startPos == TeamClass.StartPos.Near_wall){
                if(Math.abs(drivetrain.odometryClass.getEncGlobalPosition2D().getHeading()) < minTurnAngle) drivetrain.motors.setPower(0,0, 0.1);
                else drivetrain.motors.setPower(0,0,0);
            }else {
                if(Math.abs(drivetrain.odometryClass.getEncGlobalPosition2D().getHeading()) < minTurnAngle) drivetrain.motors.setPower(0,0, -0.1);
                else drivetrain.motors.setPower(0,0,0);
            }
        }
    }
}
