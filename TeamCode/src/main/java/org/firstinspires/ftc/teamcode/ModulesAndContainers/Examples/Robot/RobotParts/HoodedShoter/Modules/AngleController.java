package org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.HoodedShoter.Modules;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Config.MainFile;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Wrappers.Examples.ServoMotorWrapper;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Extenders.ExecutableCollector;

public class AngleController extends ExecutableCollector {
    public String angleServo = controlHubDevices.getServo(3);
    public AngleController(MainFile mainFile) {
        super(mainFile);
        createServoWrapperUtils();
        servosCollector.add(servoBuilder.initialize(mainFile, angleServo).setFields(60.0, 120.0).get());

        sayCreated();
    }

    public double getCurAngle(){
        if(!isInitialized) return 0;
        double angle = -((servosCollector.get(angleServo).servo.getPosition() * (MAX_ANGLE - MIN_ANGLE)) - MAX_ANGLE);

        return Math.round(angle * Math.pow(10, 2)) / Math.pow(10, 2);
    }
    public double getPos(double theta){
        //Рассчитываем угол вылета

        double alpha = Math.toRadians(theta);

        double rampAngle = Range.clip(90 - Math.toDegrees(alpha), MIN_ANGLE, MAX_ANGLE);

        double targetServoPos =  ((MAX_ANGLE - rampAngle) / (MAX_ANGLE - MIN_ANGLE) * 360) / 360;

        targetServoPos = Math.round(targetServoPos * Math.pow(10, 2)) / Math.pow(10, 2);

        //Выставляем нужную позицию
        return targetServoPos;
    }
    public ServoMotorWrapper getServo(){
        return servosCollector.get(angleServo);
    }
    @Override
    protected void executeExt(Double... args) {
        servosCollector.get(angleServo).execute(args);
    }

    @Override
    protected void showDataExt() {
        servosCollector.showData();
        telemetry.addData("curAngle", getCurAngle());
    }
}
