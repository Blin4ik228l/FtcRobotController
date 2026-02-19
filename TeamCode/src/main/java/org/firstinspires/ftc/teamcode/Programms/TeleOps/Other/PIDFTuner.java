package org.firstinspires.ftc.teamcode.Programms.TeleOps.Other;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ConstansOrMagicNumbers.kPIDS;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Joysticks.JoystickActivityClass;
import org.firstinspires.ftc.teamcode.Programms.TeleOps.TeleOpModernized;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.GeneralInformation;

@TeleOp(name = "PIDFTuner")
public class PIDFTuner extends TeleOpModernized implements kPIDS {
    private double  P = FLYWHEEL[0], I = FLYWHEEL[1], D = FLYWHEEL[2], F = FLYWHEEL[3];
    private double[] stepSize = {1, 0.1, 0.01, 0.001, 0.0001, 0.00001};
    private int stepIndex;
    private int index;
    private JoystickActivityClass joystickActivityClass2;
    @Override
    public void init() {
        generalInformation = new GeneralInformation(GeneralInformation.ProgramName.TeleOp, GeneralInformation.Color.Blue, GeneralInformation.StartPos.Nevermind);

        initAfterRobot();

        joystickActivityClass2 = opDataContainer.autoPlayerClass2.joystickActivityClass;
    }

    @Override
    public void extUpdate() {

    }

    @Override
    public void extExecute() {

        if(joystickActivityClass2.bumperLeft){
            stepIndex = Math.max(stepIndex - 1, 0);
            joystickActivityClass2.bumperLeft = false;
        }

        if(joystickActivityClass2.bumperRight){
            stepIndex = (stepIndex + 1) % stepSize.length;
            joystickActivityClass2.bumperRight = false;
        }

        if(joystickActivityClass2.triggerLeft){
            index = Math.max(index - 1, 0);
            joystickActivityClass2.triggerLeft = false;
        }

        if(joystickActivityClass2.triggerRight){
            index = (index + 1) % 4;
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

        opDataContainer.robot.hoodedShoter.motors.setPIDF(P, I, D, F);
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
