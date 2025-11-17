package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Modules.UpdatableModule;

public class TeamColor extends UpdatableModule {
    public final String red = "Red";
    public final String blue = "Blue";

    public TeamColor(String chosenColor, OpMode op){
        super(op.telemetry);
        this.chosenColor = chosenColor;

        blueWallCoord[3] = Math.atan2(blueWallCoord[1], blueWallCoord[0]);
        redWallCoord[3] = Math.atan2(redWallCoord[1], redWallCoord[0]);

        aprilTagIds = chosenColor.equals(red) ? aprilTagRedIds : aprilTagBlueIds ;
        wallCoord = chosenColor.equals(red) ? redWallCoord : blueWallCoord;
        artifactsCoord = chosenColor.equals(red) ? artifactsUnderRedWallCoord : artifactsUnderBlueWallCoord;
    }

    private String chosenColor;

    private int[] aprilTagIds;

    private double[] wallCoord;
    private double[][] artifactsCoord;

    private final int[] aprilTagBlueIds = {20};

    private final int[] aprilTagRedIds = {24};

    public int[] getTagIds(){
        return aprilTagIds;
    }
    public double[] getWallCoord(){
        return wallCoord;
    }
    public double[][] getClosestArtifacts(){
        return artifactsCoord;
    }
    public double[] getWallCoord(int id){
        if(id == aprilTagBlueIds[0]) return blueWallCoord;
        else return redWallCoord;
    }
    private final double[] blueWallCoord = new double[]{
            2.54 * (-58.3727),//X
            2.54 * (-55.6425),//Y
            2.54 * (29.5),//Z
            0//Угол относительно поля
    };
    private final double[] redWallCoord = new double[]{
            2.54 * (-58.3727),//X
            2.54 * (55.6425),//Y
            2.54 * (29.5),//Z
            0//Угол относительно поля
    };

    public final double[][]artifactsUnderBlueWallCoord = new double[][]{
            new double[]{-30.401,-133.588,6.264 , Math.toRadians(-180)}, new double[]{-30.401,-122.888,6.264 , Math.toRadians(-180)}, new double[]{-30.401,-108.188,6.264 , Math.toRadians(-180)},
            new double[]{ 29.607,-133.588,6.264 , Math.toRadians(-180)}, new double[]{29.607 ,-122.888,6.264 , Math.toRadians(-180)}, new double[]{29.607 ,-108.188,6.264 , Math.toRadians(-180)},
            new double[]{ 89.614,-133.588,6.264 , Math.toRadians(-180)}, new double[]{89.614 ,-122.888,6.264 , Math.toRadians(-180)}, new double[]{89.614 ,-108.188,6.264 , Math.toRadians(-180)}
    };

    public final double[][]artifactsUnderRedWallCoord = new double[][]{
            new double[]{-30.401,133.588,6.264 , Math.toRadians(90)}, new double[]{-30.401,122.888,6.264 , Math.toRadians(90)}, new double[]{-30.401,108.188,6.264 , Math.toRadians(90)},
            new double[]{29.607 ,133.588,6.264 , Math.toRadians(90)}, new double[]{29.607 ,122.888,6.264 , Math.toRadians(90)}, new double[]{29.607 ,108.188,6.264 , Math.toRadians(90)},
            new double[]{89.614 ,133.588,6.264 , Math.toRadians(90)}, new double[]{89.614 ,122.888,6.264 , Math.toRadians(90)}, new double[]{89.614 ,108.188,6.264 , Math.toRadians(90)}
    };

}
