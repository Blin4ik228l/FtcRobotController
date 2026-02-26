package org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.HoodedShoter.Modules;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Wrappers.Examples.ServoMotorWrapper;

public class AngleController extends ServoMotorWrapper {
    public AngleController(OpMode op, String deviceName) {
        super(op, deviceName);
    }

    public double getCurAngle(){
        if (!isInitialized) return 0;
        double angle = MAX_ANGLE - servo.getPosition() / (185 / 23) * 270;

        return Math.round(angle * Math.pow(10, 2)) / Math.pow(10, 2);
    }
    public double getPos(double theta){
        if(!isInitialized) return 0;
        //Рассчитываем угол вылета

        double alpha = Math.toRadians(theta);

        double rampAngle = Range.clip(90 - Math.toDegrees(alpha), MIN_ANGLE, MAX_ANGLE);

        double targetServoPos =  (MAX_ANGLE - rampAngle) * (185 / 23) / 270;

        targetServoPos = Math.round(targetServoPos * Math.pow(10, 2)) / Math.pow(10, 2);

        //Выставляем нужную позицию
        return targetServoPos;
    }
}
