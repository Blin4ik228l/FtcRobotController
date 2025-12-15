package org.firstinspires.ftc.teamcode.Robot.RobotParts.DrivetrainParts;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Modules.Types.UpdatableModule;

public class TeamColorClass extends UpdatableModule {
    public StartPos startPos;
    public Color color;
    public enum Color{
        Red,
        Blue
    }
    public enum StartPos{
        Near_wall,
        Far_from_wall,
        Nevermind
    }
    public TeamColorClass(Color color, StartPos startPos, OpMode op){
        super(op.telemetry);
        this.startPos = startPos;
        this.color = color;

        blueTagCoord[3] = Math.atan2(blueTagCoord[1], blueTagCoord[0]);
        redTagCoord[3] = Math.atan2(redTagCoord[1], redTagCoord[0]);

        bluePointVyrCoord[3] = Math.atan2(bluePointVyrCoord[1], bluePointVyrCoord[0]);
        redPointVyrCoord[3] = Math.atan2(redPointVyrCoord[1], redPointVyrCoord[0]);

        aprilTagIds = color == Color.Red ? aprilTagRedIds : aprilTagBlueIds ;
        tagCoord = color == Color.Red ? redTagCoord : blueTagCoord;
        pointVyr = color == Color.Red ? redPointVyrCoord : bluePointVyrCoord;
        fireZones = color == Color.Red ? fireZonesRed : fireZonesBlue;
        baseCoord = color == Color.Red ? baseRedCoord : baseBlueCoord;
        artifactsCoord = color == Color.Red ? artifactsUnderRedWallCoord : artifactsUnderBlueWallCoord;
    }
    private int[] aprilTagIds;
    private double[] tagCoord;
    private double[][] artifactsCoord;
    private double[][] fireZones;
    private double[] pointVyr;
    private double[] baseCoord;
    private final int[] aprilTagBlueIds = {20};
    private final int[] aprilTagRedIds = {24};

    public int[] getAprilTagIds() {
        return aprilTagIds;
    }
    public double[] getPointVyr(){
        return pointVyr;
    }
    public double[] getTagCoord(){
        return tagCoord;
    }
    public double[] getTagCoord(int id){
        if(id == aprilTagBlueIds[0]) return blueTagCoord;
        else return redTagCoord;
    }
    public double[][] getClosestArtifacts(){
        return artifactsCoord;
    }
    public double[][] getFireZones(){
        return fireZones;
    }
    public double[] getBaseCoord(){return baseCoord;}
    public final double blueTagCoord[] = new double[]{
            2.54 * (-58.3727),//X
            2.54 * (-55.6425),//Y
            2.54 * (29.5),//Z
            0
    };
    public final double redTagCoord[] = new double[]{
            2.54 * (-58.3727),//X
            2.54 * (55.6425),//Y
            2.54 * (29.5),//Z
            0
    };
    private final double[] bluePointVyrCoord = new double[]{
//            -158,//X
//            -178,//Y
            -180,//X
            -180,//Y
            2.54 * (29.5),//Z
            0//Угол относительно поля
    };
    private final double[] redPointVyrCoord = new double[]{
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

    public final double[] baseBlueCoord = new double[]{
            0,0,0
    };
    public final double[] baseRedCoord = new double[]{
      0,0,0
    };

    @Override
    public void showData() {
        telemetry.addLine("===TEAM COLOR===");
        telemetry.addData("Color", color.toString());
        telemetry.addData("Start pos", startPos.toString());
        telemetry.addLine();
    }
}
