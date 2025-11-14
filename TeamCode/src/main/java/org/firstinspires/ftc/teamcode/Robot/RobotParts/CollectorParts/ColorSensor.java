package org.firstinspires.ftc.teamcode.Robot.RobotParts.CollectorParts;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Modules.Module;

public class ColorSensor extends Module {
    public ColorSensor(OpMode op){
        super(op.telemetry);

        colorSensor = op.hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        colorSensor1 = op.hardwareMap.get(NormalizedColorSensor.class, "sensor_color2");

        colorSensor.setGain(gain);
        colorSensor1.setGain(gain);

        relativeLayoutId = op.hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", op.hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) op.hardwareMap.appContext).findViewById(relativeLayoutId);

        telemetry.addLine("ColorSensor Inited");
    }
    private final int relativeLayoutId;
    private final NormalizedColorSensor colorSensor, colorSensor1;
    private final View relativeLayout;
    private float gain = 3f;
    private final float[] hsvValues = new float[3];
    public NormalizedRGBA colors, colors1;
    public float red;
    public float blue;
    public float green;
    public float alpha;
    public float red1;
    public float blue1;
    public float green1;
    public float alpha1;
    public double currentArtifact, currentArtifact1;
    public double lastSeenArtifact = 0;
    public double curDistance,curDistance1;
    int count;

    public void execute(){
        colors = colorSensor.getNormalizedColors();
        colors1 = colorSensor1.getNormalizedColors();

        Color.colorToHSV(colors.toColor(), hsvValues);

//                relativeLayout.setBackgroundColor(Color.HSVToColor(hsvValues));

        red = colors.red;
        blue = colors.blue;
        green = colors.green;
        alpha = colors.alpha;

        red1 = colors1.red;
        blue1 = colors1.blue;
        green1 = colors1.green;
        alpha1 = colors1.alpha;

        currentArtifact = getDominantColor();
        currentArtifact1 = getDominantColor2();

        if(currentArtifact != 0 ) lastSeenArtifact = currentArtifact;

        curDistance = getDistance();
        curDistance1 = getDistance2();
    }
    public int getDominantColor(){
        if(red > blue && red > green && red > 0.04) {
            return 2;}
        else if(blue > red && blue > green && blue > 0.04) {
            return 2;}
        else if(green > red && green > blue && green > 0.05) {
            return 1;}
        else return 0;
    }
    public int getDominantColor2(){
        if(red1 > blue1 && red1 > green1 && red1 > 0.05) {
            return 2;}
        else if(blue1 > red1 && blue1 > green1 && blue1 > 0.05) {
            return 2;}
        else if(green1 > red1 && green1 > blue1 && green1 > 0.1) {
            return 1;}
        else return 0;
    }
    public double getDistance(){
        return ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM);
    }
    public double getDistance2(){
        return ((DistanceSensor) colorSensor1).getDistance(DistanceUnit.CM);
    }
    public String getColorFromNumber(double number){
        return number == 2 ? "Purple" : number == 1 ? "Green" : "Empty";
    }
    public void showData(){
        telemetry.addLine("Color-sensor data")
                .addData("\nCurrent see color", "%s %n", getColorFromNumber(currentArtifact))
                .addData("Current2 see color", "%s %n", getColorFromNumber(currentArtifact1))
                .addData("Distance in CM", "%.1f %n", curDistance)
                .addData("Distance2 in CM", "%.1f %n", curDistance1)
                .addData("count", "%s %n", count);
        telemetry.addLine("Values from sensor %n")
                .addData("Red", "%.3f %n", red)
                .addData("Green", "%.3f %n", green)
                .addData("Blue", "%.3f %n", blue)
                .addData("Red1", "%.3f %n", red1)
                .addData("Green1", "%.3f %n", green1)
                .addData("Blue1", "%.3f %n", blue1);
        telemetry.addLine();

    }

}