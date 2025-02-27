package org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules.Physical.Singles;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules.Module;

public class Distance implements Module {
    private OpMode op;

    private DistanceSensor sensorDistance;

    public Distance(OpMode op){
        this.op = op;
    }

    @Override
    public void init() {
        sensorDistance = op.hardwareMap.get(DistanceSensor.class, "sensor_distance");
    }



    public DistanceSensor getSensorDistance() {
        return sensorDistance;
    }

    public double getDistance(){
        return sensorDistance.getDistance(DistanceUnit.CM);
    }
}
