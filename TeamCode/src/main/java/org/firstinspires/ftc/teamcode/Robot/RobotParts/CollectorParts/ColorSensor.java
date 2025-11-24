package org.firstinspires.ftc.teamcode.Robot.RobotParts.CollectorParts;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Modules.UpdatableModule;

public class ColorSensor extends UpdatableModule {
    public ColorSensor(OpMode op){
        super(op.telemetry);

        colorSensor = op.hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        colorSensor1 = op.hardwareMap.get(NormalizedColorSensor.class, "sensor_color2");

        colorSensor.setGain(gain);
        colorSensor1.setGain(gain);

        relativeLayoutId = op.hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", op.hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) op.hardwareMap.appContext).findViewById(relativeLayoutId);

        timeFromDetect = new ElapsedTime();

        telemetry.addLine("ColorSensor Inited");
    }
    private final int relativeLayoutId;
    private final NormalizedColorSensor colorSensor, colorSensor1;
    private final View relativeLayout;
    private float gain = 3f;
    private final float[] hsvValues = new float[3];
    public NormalizedRGBA sensor0Colors, sensor2Colors;
    public float red0, blue0, green0, alpha0;
    public float red2, blue2, green2, alpha2;
    public double sensor0FoundedColor, sensor2FoundedColor;
    public double sensor0Distance, sensor2Distance;
    public double artifactColor;
    public ColorSensorState colorState;
    public ElapsedTime timeFromDetect;

    public enum ColorSensorState{
        No_Artifact_Detected,
        Artifact_Detected
    }
    @Override
    public void update(){
        updateColorSensor0();
        updateColorSensor2();

        Color.colorToHSV(sensor0Colors.toColor(), hsvValues);

//      relativeLayout.setBackgroundColor(Color.HSVToColor(hsvValues));

        updateClassState();
    }
    public void updateColorSensor0(){
        sensor0Colors = colorSensor.getNormalizedColors();

        red0 = sensor0Colors.red;
        blue0 = sensor0Colors.blue;
        green0 = sensor0Colors.green;
        alpha0 = sensor0Colors.alpha;

        updateSensor0DominantColor();
        updateDistance0();
    }
    public void updateColorSensor2(){
        sensor2Colors = colorSensor1.getNormalizedColors();

        red2 = sensor2Colors.red;
        blue2 = sensor2Colors.blue;
        green2 = sensor2Colors.green;
        alpha2 = sensor2Colors.alpha;

        updateSensor2DominantColor();
        updateDistance2();

        updateArtifactColor();
    }
    public void updateSensor0DominantColor(){
        if(red0 > blue0 && red0 > green0 && red0 > 0.04) {
            sensor0FoundedColor =  2;}
        else if(blue0 > red0 && blue0 > green0 && blue0 > 0.04) {
            sensor0FoundedColor =  2;}
        else if(green0 > red0 && green0 > blue0 && green0 > 0.05) {
            sensor0FoundedColor =  1;}
        else sensor0FoundedColor =  0;
    }
    public void updateSensor2DominantColor(){
        if(red2 > blue2 && red2 > green2 && red2 > 0.05) {
            sensor2FoundedColor =  2;}
        else if(blue2 > red2 && blue2 > green2 && blue2 > 0.05) {
            sensor2FoundedColor =  2;}
        else if(green2 > red2 && green2 > blue2 && green2 > 0.1) {
            sensor2FoundedColor =  1;}
        else sensor2FoundedColor = 0;
    }
    public void updateDistance0(){
        sensor0Distance = ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM);
    }
    public void updateDistance2(){
        sensor2Distance = ((DistanceSensor) colorSensor1).getDistance(DistanceUnit.CM);
    }

    public void updateClassState(){
        if((sensor0FoundedColor != 0 || sensor2FoundedColor != 0) && sensor0Distance < 10) {
            colorState = ColorSensorState.Artifact_Detected;
        }
        else {
            colorState = ColorSensorState.No_Artifact_Detected;
            timeFromDetect.reset();
        }
    }
    public void updateArtifactColor(){
        artifactColor = sensor0FoundedColor != 0 ? sensor0FoundedColor : sensor2FoundedColor;
    }
    public String getColorFromNumber(double number){
        return number == 2 ? "Purple" : number == 1 ? "Green" : "Empty";
    }
    public void showData(){
        telemetry.addLine("=== COLOR SENSOR ===");
        telemetry.addData("Colors", "Cur:%s Cur2:%s", getColorFromNumber(sensor0FoundedColor), getColorFromNumber(sensor2FoundedColor));
        telemetry.addData("Distance", "%.1fcm", sensor0Distance);
        telemetry.addData("RGB1", "R:%.3f G:%.3f B:%.3f", red0, green0, blue0);
        telemetry.addData("RGB2", "R:%.3f G:%.3f B:%.3f", red2, green2, blue2);
        telemetry.addData("Color sensor state", colorState.toString());
        telemetry.addLine();
    }

}