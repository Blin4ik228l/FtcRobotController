package org.firstinspires.ftc.teamcode.Robot.RobotParts.Collector.CollectorParts;

import android.app.Activity;
import android.graphics.Color;
import android.widget.RelativeLayout;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Modules.Types.UpdatableModule;

public class ColorSensorClass extends UpdatableModule {
    private final NormalizedColorSensor colorSensorNear, colorSensorFar;
    private RelativeLayout relativeLayout;
    private float gain = 15f;
    public ColorSensorState colorState;
    private ElapsedTime timeFromDetect;
    public ColorSensorClass(OpMode op){
        super(op.telemetry);

        colorSensorNear = op.hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        colorSensorFar = op.hardwareMap.get(NormalizedColorSensor.class, "sensor_color2");

        colorSensorNear.setGain(gain);
        colorSensorFar.setGain(gain);

        colorState = ColorSensorState.No_Artifact_Detected;

        timeFromDetect = new ElapsedTime();

        telemetry.addLine("ColorSensor Inited");
    }

    public NormalizedRGBA sensorNearCol, sensorFarCol;
    private final float[] hsvValues = new float[3];
    public double r, g, b;
    public double dist;
    public double sensorNearDist, sensorFarDist;
    private int artifactColor;
    public enum ColorSensorState{
        No_Artifact_Detected,
        Artifact_Detected
    }

    public ElapsedTime getTimeFromDetect() {
        return timeFromDetect;
    }

    private void initLayout(OpMode op) {
        try {
            // Самый прямой и надежный способ
            int layoutId = com.qualcomm.ftcrobotcontroller.R.id.RelativeLayout;

            Activity activity = (Activity) op.hardwareMap.appContext;
            relativeLayout = activity.findViewById(layoutId);

            if (relativeLayout == null) {
                op.telemetry.log().add("RelativeLayout not found!");
            }else
            {
                // Меняем цвет фона
                relativeLayout.setBackgroundColor(Color.RED);
            }
        } catch (Exception e) {
            op.telemetry.log().add("Failed to get RelativeLayout: " + e);
        }
    }

    public int getArtifactColor() {
        return artifactColor;
    }

    @Override
    public void update(){
        updateColorSensor0();
        updateColorSensor2();

//        Color.colorToHSV(sensorNearCol.toColor(), hsvValues);
//        if (relativeLayout != null && relativeLayout.isShown()) {
//            relativeLayout.setBackgroundColor(Color.HSVToColor(hsvValues));
//        }
        updateInAll();
    }
    public void updateColorSensor0(){
        sensorNearCol = colorSensorNear.getNormalizedColors();

        updateDistance0();
    }
    public void updateColorSensor2(){
        sensorFarCol = colorSensorFar.getNormalizedColors();

        updateDistance2();
    }

    public void updateInAll(){
        r = (sensorNearCol.red + sensorFarCol.red) / 2.0;
        g = (sensorNearCol.green + sensorFarCol.green) / 2.0;
        b = (sensorNearCol.blue + sensorFarCol.blue) / 2.0;

        dist = (sensorNearDist + sensorFarDist) / 2.0;

        int foundColor;
        if(r > g && r > b && r > 0.075) {
            foundColor =  2;}
        else if(b > r && b > g && b > 0.040) {
            foundColor =  2;}
        else if(g > r && g > b && g > 0.078) {
            foundColor =  1;}

        else foundColor =  0;

        int detectedDist = 5;

        if(foundColor != 0 && dist < detectedDist) {
            colorState = ColorSensorState.Artifact_Detected;
            artifactColor = foundColor;
        }
        else {
            colorState = ColorSensorState.No_Artifact_Detected;
            timeFromDetect.reset();
        }
    }
//
    public void updateDistance0(){
        sensorNearDist = ((DistanceSensor) colorSensorNear).getDistance(DistanceUnit.CM);
    }
    public void updateDistance2(){
        sensorFarDist = ((DistanceSensor) colorSensorFar).getDistance(DistanceUnit.CM);
    }

//    public void updateSensor0DominantColor(){
//        if(red0 > blue0 && red0 > green0 && red0 > 0.037) {
//            sensor0FoundedColor =  2;}
//        else if(blue0 > red0 && blue0 > green0 && blue0 > 0.04) {
//            sensor0FoundedColor =  2;}
//        else if(green0 > red0 && green0 > blue0 && green0 > 0.096) {
//            sensor0FoundedColor =  1;}
//        else sensor0FoundedColor =  0;
//    }
//    public void updateSensor2DominantColor(){
//        if(red2 > blue2 && red2 > green2 && red2 > 0.037) {
//            sensor2FoundedColor = 2;}
//        else if(blue2 > red2 && blue2 > green2 && blue2 > 0.04) {
//            sensor2FoundedColor = 2;}
//        else if(green2 > red2 && green2 > blue2 && green2 > 0.070) {
//            sensor2FoundedColor = 1;}
//        else sensor2FoundedColor = 0;
//    }
//    public void updateClassState(){
//        //TODO
//        int detectedDist = 6;
//
//        if((sensor0FoundedColor != 0 || sensor2FoundedColor != 0) && (sensor0Distance < detectedDist || sensor2Distance < detectedDist)) {
//            colorState = ColorSensorState.Artifact_Detected;
//            updateArtifactColor();
//        }
//        else {
//            colorState = ColorSensorState.No_Artifact_Detected;
//            timeFromDetect.reset();
//        }
//    }
//    public void updateArtifactColor(){
//        artifactColor = sensor0FoundedColor != 0 ? sensor0FoundedColor : sensor2FoundedColor;
//    }
    public String getColorFromNumber(double number){
        return number == 2 ? "Purple" : number == 1 ? "Green" : "Empty";
    }
    public void showData(){
        telemetry.addLine("===COLOR SENSOR===");
        telemetry.addData("Colors",  getColorFromNumber(artifactColor));
        telemetry.addData("Distance", dist);
        telemetry.addData("Time", timeFromDetect.seconds());
        telemetry.addData("Colors", "R:%.3f G:%.3f B:%.3f", r, g, b);
        telemetry.addData("Color sensor state", colorState.toString());
        telemetry.addLine();
    }

}