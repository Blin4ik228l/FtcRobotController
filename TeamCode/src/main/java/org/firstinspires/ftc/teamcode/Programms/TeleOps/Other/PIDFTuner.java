package org.firstinspires.ftc.teamcode.Programms.TeleOps.Other;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Programms.TeleOps.TeleOpModernized;
import org.firstinspires.ftc.teamcode.Robot.RobotClass;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DriveTrain.DrivetrainParts.TeamClass;

@TeleOp(name = "PIDFTuner")
public class PIDFTuner extends TeleOpModernized {
    //    private double  P = 20.5, I, D = 1.5, F = 0.23;
//    private double  P = 19, I = 0.11, D = 3.0, F = 0.41;
//    private double  P = 10, I = 0, D = 3.0, F = 0;
//    private double  P = 17, I = 0.2, D = 3.0, F = 0.08;
private double  P = 0, I = 0, D = 0, F = 0;
    private double[] stepSize = {1, 0.1, 0.01, 0.001, 0.0001, 0.00001};
    private int stepIndex;
    private int index;

    @Override
    public void init() {
        robot = new RobotClass(TeamClass.Color.Blue, TeamClass.StartPos.Nevermind, this);

        initAfterRobot();
    }

    @Override
    public void extExecute() {
        if(joystickActivityClass2.bumperLeft){
            stepIndex = (stepIndex + 1) % stepSize.length;
            joystickActivityClass2.bumperLeft = false;
        }

        if(joystickActivityClass2.bumperRight){
            stepIndex = Math.max(stepIndex - 1, 0);
            joystickActivityClass2.bumperRight = false;
        }

        if(joystickActivityClass2.triggerLeft){
            index = (index + 1) % 4;
            joystickActivityClass2.triggerLeft = false;
        }

        if(joystickActivityClass2.triggerRight){
            index = Math.max(index - 1, 0);
            joystickActivityClass2.triggerRight = false;
        }

        if(joystickActivityClass2.dpad_Up){
            switch (index){
                case 0:
                    P += stepSize[stepIndex];
                    break;
                case 1:
                    I += stepSize[stepIndex];
                    break;
                case 2:
                    D += stepSize[stepIndex];
                    break;
                case 3:
                    F += stepSize[stepIndex];
                    break;
            }
            joystickActivityClass2.dpad_Up = false;
        }

        if(joystickActivityClass2.dpad_Down){
            switch (index){
                case 0:
                    P = Math.max(P - stepSize[stepIndex], 0);
                    break;
                case 1:
                    I = Math.max(I - stepSize[stepIndex], 0);
                    break;
                case 2:
                    D = Math.max(D - stepSize[stepIndex], 0);
                    break;
                case 3:
                    F = Math.max(F - stepSize[stepIndex], 0);
                    break;
            }
            joystickActivityClass2.dpad_Down = false;
        }

        robot.collector.motors.setPIDF(P, I, D, F);
    }

    @Override
    public void extShow() {
        telemetry.addLine();
        telemetry.addData("P: ", P);
        telemetry.addData("I: ", I);
        telemetry.addData("D: ", D);
        telemetry.addData("F: ", F);
        telemetry.addData("Step index and size", "in: %s sz: %s", stepIndex, stepSize[stepIndex]);
        telemetry.addData("Index", index);
    }
}
