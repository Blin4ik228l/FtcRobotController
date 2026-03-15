package org.firstinspires.ftc.teamcode.MainParts.Examples.Wrappers.Examples;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MainParts.Modules.Extenders.Extenders2.ExecutableModule;

public class ServoMotorWrapper extends ExecutableModule {
    private ElapsedTime signalTime;
    public Servo servo;
    public ServoMotorWrapper(String searchingDevice) {
        super(searchingDevice);

        try {
            servo = hardwareMap.get(Servo.class, searchingDevice);
        }catch (Exception e){
            isInitialized = false;
        }

        signalTime = new ElapsedTime();
        sayInited();
    }
    private double servoSpeed;
    private double servoMaxAngle;
    private double delayTime;
    @Override
    protected void executeExt(Double... args) {
        //По сути метод нужен для серваков без обратной связи
        double position = args[0];
        if (position != servo.getPosition()) {
            delayTime = (Math.abs(position - servo.getPosition()) * servoMaxAngle) / servoSpeed;
            signalTime.reset();
        }
        //Вычисляем "путь" до позиции, а после расщётное время
        servo.setPosition(position);
    }

    public boolean isBusy() {
        return signalTime.seconds() < delayTime * 8.0;
    }

    @Override
    public void showDataExt() {
        telemetry.addData("Position", servo.getPosition());
        telemetry.addData("signal Time", signalTime.seconds());
        telemetry.addData("delay", delayTime);
    }

    public static class InnerBuilder extends Builder<ServoMotorWrapper>{

        @Override
        public InnerBuilder initialize(String searchingDevice) {
            wrapper = new ServoMotorWrapper(searchingDevice);
            return this;
        }

        @Override
        public InnerBuilder setFields(Double... args) {
            wrapper.servoSpeed = args[0];
            wrapper.servoMaxAngle = args[1];
            return this;
        }
    }

    public static class InnerCollector extends CollectorBuilder<ServoMotorWrapper>{

        @Override
        public InnerCollector add(ServoMotorWrapper wrapper) {
            wrappers.put(wrapper.searchingDevice, wrapper);
            return this;
        }

        @Override
        public void showData() {
            for (ServoMotorWrapper wrapper : wrappers.values()) {
                wrapper.showData();
            }
        }
    }
}
