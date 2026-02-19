package org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Wrappers;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.HardwareBuilder;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Module;

import java.util.HashMap;

public class ServoMotorWrapper extends Module {
    private ElapsedTime signalTime;
    public Servo servo;
    public ServoMotorWrapper(OpMode op, String deviceName) {
        super(op);
        servo = hardwareMap.get(Servo.class, deviceName);

        signalTime = new ElapsedTime();
    }
    private double servoSpeed;
    private double servoMaxAngle;
    private double delayTime;
    private boolean isBusy;
    public boolean setSignal(double position){
        //По сути метод нужен для серваков без обратной связи
        if (position != servo.getPosition()) signalTime.reset();

        servo.setPosition(position);
        //Вычисляем "путь" до позиции, а после расщётное время
        delayTime = Math.abs(servoMaxAngle * position - servo.getPosition() * servoMaxAngle) * servoSpeed;
        isBusy = signalTime.seconds() > delayTime;
        return isBusy;
    }

    public boolean isBusy(){
        return signalTime.seconds() > delayTime;
    }
    //Этот метод по - хорошему использовать с обратной связью
    public void setPosition(double position){
        servo.setPosition(position);
    }

    @Override
    public void showData() {
        telemetry.addLine("===" + servo.getDeviceName() + "===");
        telemetry.addData("Position", servo.getPosition());
        telemetry.addLine();
    }
    public static class Builder extends HardwareBuilder {
        private HashMap<String, ServoMotorWrapper> servos = new HashMap<>();
        @Override
        public Builder initialize(OpMode op, String deviceName) {
            servos.put(deviceName, new ServoMotorWrapper(op, deviceName));
            return this;
        }
        public Builder setFields(String deviceName, double servoSpeed, double servoMaxAngle){
            servos.get(deviceName).servoSpeed = servoSpeed;
            servos.get(deviceName).servoMaxAngle = servoMaxAngle;
            return this;
        }
        public boolean setSignal(String deviceName, double position){
            return servos.get(deviceName).setSignal(position);
        }

        public void setPosition(String deviceName, double position){
            servos.get(deviceName).setPosition(position);
        }
    }
}
