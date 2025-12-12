package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Modules.Types.UpdatableModule;

public class TeamColor extends UpdatableModule {
    public StartPos startPos;
    public Color color;
    public enum Color{
        Red,
        Blue
    }
    public enum StartPos{
        Near_wall,
        Far_from_wall
    }
    public TeamColor(Color color, StartPos startPos,OpMode op){
        super(op.telemetry);
        this.startPos = startPos;
        this.color = color;

        blueWallCoord[3] = Math.atan2(blueWallCoord[1], blueWallCoord[0]);
        redWallCoord[3] = Math.atan2(redWallCoord[1], redWallCoord[0]);

        aprilTagIds = color == Color.Red ? aprilTagRedIds : aprilTagBlueIds ;
        wallCoord = color == Color.Red ? redWallCoord : blueWallCoord;
        artifactsCoord = color == Color.Red ? artifactsUnderRedWallCoord : artifactsUnderBlueWallCoord;
        fireZones = color == Color.Red ? fireZonesRed : fireZonesBlue;

    }
    private int[] randomizedArtifact = new int[3];
    /*0 - в массиве это зелёный шар
     * 1 - в массиве это фиолетовый*/

    public void setRandomizedArtifact(int[] randomizedArtifact) {
        this.randomizedArtifact = randomizedArtifact;
    }

    public int[] getRandomizedArtifact() {
        return randomizedArtifact;
    }

    private int[] aprilTagIds;
    private double[] wallCoord;
    private double[][] artifactsCoord;
    private double[][] fireZones;
    private final int[] aprilTagBlueIds = {20};
    private final int[] aprilTagRedIds = {24};

    public int[] getAprilTagIds() {
        return aprilTagIds;
    }

    public double[] getWallCoord(){
        return wallCoord;
    }
    public double[][] getClosestArtifacts(){
        return artifactsCoord;
    }
    public double[][] getFireZones(){
        return fireZones;
    }
    public double[] getWallCoord(int id){
        if(id == aprilTagBlueIds[0]) return blueWallCoord;
        else return redWallCoord;
    }
    private final double[] blueWallCoord = new double[]{
//            -158,//X
//            -178,//Y
            -180,//X
            -180,//Y
            2.54 * (29.5),//Z
            0//Угол относительно поля
    };
    public final double getBlueWallCoord[] = new double[]{
            2.54 * (-58.3727),//X
            2.54 * (-55.6425),//Y
            2.54 * (29.5)//Z
    };
    public final double getRedWallCoord[] = new double[]{
            2.54 * (-58.3727),//X
            2.54 * (55.6425),//Y
            2.54 * (29.5)//Z
    };
    private final double[] redWallCoord = new double[]{
            -180,//X
            180,//Y
            2.54 * (29.5),//Z
            0//Угол относительно поля
    };

    public final double[][]artifactsUnderBlueWallCoord = new double[][]{
            //                GREEN                                                        PURPLE                                                            PURPLE
            new double[]{1 ,-30.401,-133.588,6.264 , Math.toRadians(-90)}, new double[]{2, -30.401,-122.888,6.264 , Math.toRadians(-90)}, new double[]{2, -30.401,-108.188,6.264 , Math.toRadians(-180)},
            //                PURPLE                                                        GREEN                                                            PURPLE
            new double[]{2,  29.607,-133.588,6.264 , Math.toRadians(-90)}, new double[]{1, 29.607 ,-122.888,6.264 , Math.toRadians(-90)}, new double[]{2, 29.607 ,-108.188,6.264 , Math.toRadians(-180)},
            //                PURPLE                                                        PURPLE                                                            GREEN
            new double[]{2, 89.614,-133.588,6.264 , Math.toRadians(-90)}, new double[]{2, 89.614 ,-122.888,6.264 , Math.toRadians(-90)}, new double[]{1, 89.614 ,-108.188,6.264 , Math.toRadians(-180)}
    };

    public final double[][]artifactsUnderRedWallCoord = new double[][]{
            new double[]{1, -30.401,108.188,6.264 , Math.toRadians(90)}, new double[]{2, -30.401,122.888,6.264 , Math.toRadians(90)}, new double[]{2, -30.401,133.588,6.264 , Math.toRadians(90)},
            new double[]{2, 29.607 ,108.188,6.264 , Math.toRadians(90)}, new double[]{1, 29.607 ,122.888,6.264 , Math.toRadians(90)}, new double[]{2, 29.607 ,133.588,6.264 , Math.toRadians(90)},
            new double[]{2, 89.614 ,108.188,6.264 , Math.toRadians(90)}, new double[]{2, 89.614 ,122.888,6.264 , Math.toRadians(90)}, new double[]{1, 89.614 ,133.588,6.264 , Math.toRadians(90)},
    };

    public final double[][] fireZonesRed = new double[][]{
            new double[]{-61, 61}, new double[]{150, 15}
    };
    public final double[][] fireZonesBlue = new double[][]{
            new double[]{-61, -61}, new double[]{150, -15}
    };
    public String getColorFromNumber(int number){
        return number == 2 ? "Purple" : number == 1 ? "Green" : "Empty";
    }
    @Override
    public void showData() {
        telemetry.addLine("===TEAMCOLOR CLASS===");
        telemetry.addData("Color", color.toString());
        telemetry.addData("Start pos", startPos.toString());
        telemetry.addData("Randomized artifacts:", "%s %s %s", getColorFromNumber(randomizedArtifact[0]), getColorFromNumber(randomizedArtifact[1]), getColorFromNumber(randomizedArtifact[2]));
        telemetry.addLine();
    }
}
