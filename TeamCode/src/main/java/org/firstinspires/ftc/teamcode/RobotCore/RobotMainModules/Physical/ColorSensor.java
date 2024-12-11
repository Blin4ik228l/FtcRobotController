package org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules.Physical;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules.Module;
//SensorColor.java
public class ColorSensor implements Module {
    private final OpMode op;
    private NormalizedColorSensor sensorColor;

    // TODO реализовать методы для датчика
    public ColorSensor(OpMode op){
        this.op = op;
    }

    @Override
    public void init() {
        this.sensorColor = op.hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
    }

    public NormalizedColorSensor getSensorColor() {
        return sensorColor;
    }
}
