package org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.HoodedShoter.Modules;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Wrappers.Examples.ServoMotorWrapper;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Module;
import org.opencv.dnn.Model;

public class AngleController extends Module {
    public String angleServo = expansionHubDevices.getServo0(0);
    public ServoMotorWrapper servoMotorWrapper;
    public AngleController(OpMode op) {
        super(op);
        servoMotorWrapper = new ServoMotorWrapper(op, angleServo);
        sayInited();
    }

    public double getCurAngle(){
        double angle = MAX_ANGLE - servoMotorWrapper.servo.getPosition() / (185 / 23) * 270;

        return Math.round(angle * Math.pow(10, 2)) / Math.pow(10, 2);
    }
    public double getPos(double theta){
        //Рассчитываем угол вылета

        double alpha = Math.toRadians(theta);

        double rampAngle = Range.clip(90 - Math.toDegrees(alpha), MIN_ANGLE, MAX_ANGLE);

        double targetServoPos =  (MAX_ANGLE - rampAngle) * (185 / 23) / 270;

        targetServoPos = Math.round(targetServoPos * Math.pow(10, 2)) / Math.pow(10, 2);

        //Выставляем нужную позицию
        return targetServoPos;
    }

    @Override
    protected void showData() {

    }
}
