package org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Wrappers;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.HardwareBuilder;

import java.util.HashMap;

public class ServoMotorWrapper extends DeviceWrapper {
    private ElapsedTime signalTime;
    public Servo servo;
    public ServoMotorWrapper(OpMode op, String deviceName) {
        super(op);

        this.deviceName = deviceName;
        try {
            servo = hardwareMap.get(Servo.class, deviceName);
        }catch (Exception e){
            isInitialized = false;
        }

        signalTime = new ElapsedTime();
        sayInited();
    }
    private double servoSpeed;
    private double servoMaxAngle;
    private double delayTime;
    private boolean isBusy;
    public boolean setSignal(double position){
        if(!isInitialized) return false;
        //По сути метод нужен для серваков без обратной связи
        if (position != servo.getPosition()) signalTime.reset();

        servo.setPosition(position);
        //Вычисляем "путь" до позиции, а после расщётное время
        delayTime = Math.abs(servoMaxAngle * position - servo.getPosition() * servoMaxAngle) * servoSpeed;
        isBusy = signalTime.seconds() > delayTime;
        return isBusy;
    }

    public boolean isBusy(){
        if(!isInitialized) return false;
        return signalTime.seconds() > delayTime;
    }
    //Этот метод по - хорошему использовать с обратной связью
    public void setPosition(double position){
        if(!isInitialized) return;
        servo.setPosition(position);
    }

    @Override
    public void showData() {

        if(!isInitialized) telemetry.addLine("servo" + " " + deviceName + "Not Found/Attached");
        else {
            telemetry.addLine("===" + servo.getDeviceName() + "===");
            telemetry.addData("Position", servo.getPosition());
        }
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
