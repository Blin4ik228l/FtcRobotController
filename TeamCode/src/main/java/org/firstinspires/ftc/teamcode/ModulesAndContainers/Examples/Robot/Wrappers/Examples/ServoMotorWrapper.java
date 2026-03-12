package org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Wrappers.Examples;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Config.MainFile;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Wrappers.HardwareBuilder;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.DeviceTree.DeviceManager;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Extenders.Extenders2.ExecutableModule;

public class ServoMotorWrapper extends ExecutableModule {
    private ElapsedTime signalTime;
    public Servo servo;
    public ServoMotorWrapper(MainFile mainFile, String searchingDevice) {
        super(mainFile, searchingDevice);

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
    private boolean isBusy;
    @Override
    protected void executeExt(Double... args) {
        //По сути метод нужен для серваков без обратной связи
        double position = args[0];
        if (position != servo.getPosition()) {
            delayTime = Math.abs(servoMaxAngle * position - servo.getPosition() * servoMaxAngle) / servoSpeed;
            signalTime.reset();
        }
        //Вычисляем "путь" до позиции, а после расщётное время
        servo.setPosition(position);
    }

    public boolean isBusy() {
        return signalTime.seconds() < delayTime * 5.0;
    }

    @Override
    public void showDataExt() {
        telemetry.addData("Position", servo.getPosition());
        telemetry.addData("signal Time", signalTime.seconds());
        telemetry.addData("delay", delayTime);
    }

    public static class InnerBuilder extends Builder<ServoMotorWrapper>{

        @Override
        public InnerBuilder initialize(MainFile mainFile, String searchingDevice) {
            wrapper = new ServoMotorWrapper(mainFile, searchingDevice);
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
