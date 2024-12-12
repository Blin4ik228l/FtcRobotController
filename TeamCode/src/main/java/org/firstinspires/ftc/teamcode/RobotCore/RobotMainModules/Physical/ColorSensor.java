package org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules.Physical;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules.Module;
//SensorColor.java
public class ColorSensor implements Module {
    private final OpMode op;
    private NormalizedColorSensor sensorColor;
    private View relativeLayout;
    private int relativeLayoutId;

    // TODO реализовать методы для датчика
    public ColorSensor(OpMode op){
        this.op = op;
    }

    @Override
    public void init() {
        this.sensorColor = op.hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
        this.relativeLayoutId = op.hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", op.hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) op.hardwareMap.appContext).findViewById(relativeLayoutId);

    }

    public NormalizedColorSensor getSensorColor() {
        return sensorColor;
    }

    public int getRelativeLayoutId() {
        return relativeLayoutId;
    }

    public View getRelativeLayout() {
        return relativeLayout;
    }
}
