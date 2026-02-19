package org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples;

import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.DriveTrain.GameState;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.DriveTrain.Odometry.Parts.MathUtils.Dot;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.DriveTrain.TeamAliance;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.ZonesDataClass;

import java.util.ArrayList;
import java.util.List;

public class GeneralInformation {
    public GeneralObjects generalObjects;
    public StartPos startPos;
    public TeamAliance teamAliance;
    public ProgramName programName;
    public GameState gameState;

    public GeneralInformation(ProgramName programName, TeamAliance teamAliance, StartPos startPos){
        this.programName = programName;
        this.teamAliance = teamAliance;
        this.startPos = startPos;

        this.generalObjects = new GeneralObjects();
    }

    public enum Color{
        Red,
        Blue
    }
    public enum StartPos{
        Near_wall,
        Far_from_wall,
        Nevermind
    }
    public enum ProgramName{
        Auto,
        TeleOp
    }
    public class GeneralObjects {
        public GeneralObjects() {
            blueTagCoord[3] = Math.atan2(blueTagCoord[1], blueTagCoord[0]);
            redTagCoord[3] = Math.atan2(redTagCoord[1], redTagCoord[0]);

            bluePointVyrCoord[3] = Math.atan2(bluePointVyrCoord[1], bluePointVyrCoord[0]);
            redPointVyrCoord[3] = Math.atan2(redPointVyrCoord[1], redPointVyrCoord[0]);

//        aprilTagIds = color == Color.Red ? aprilTagRedIds : aprilTagBlueIds ;
//        tagCoord = color == Color.Red ? redTagCoord : blueTagCoord;
//        pointVyr = color == Color.Red ? redPointVyrCoord : bluePointVyrCoord;
//
//
//        baseCoord = color == Color.Red ? baseRedCoord : baseBlueCoord;
//        artifactsCoord = color == Color.Red ? artifactsUnderRedWallCoord : artifactsUnderBlueWallCoord;
        }

        public int[] aprilTagIds;
        private double[] tagCoord;
        private ArrayList<double[]> artifactsCoord;
        private double[] pointVyr;
        private double[] baseCoord;
        public final int[] aprilTagBlueIds = {20};
        public final int[] aprilTagRedIds = {24};

        public int[] getAprilTagIds() {
            return aprilTagIds;
        }

        public double[] getPointVyr() {
            return pointVyr;
        }

        public double[] getTagCoord() {
            return tagCoord;
        }

        public double[] getTagCoord(int id) {
            if (id == aprilTagBlueIds[0]) return blueTagCoord;
            else return redTagCoord;
        }

        public ArrayList<double[]> getClosestArtifacts() {
            return artifactsCoord;
        }

        public double[] getBaseCoord() {
            return baseCoord;
        }

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
                -155,
                -155,
                2.54 * (29.5),//Z
                0//Угол относительно поля
        };
        private final double[] redPointVyrCoord = new double[]{
                -155,
                155,
                2.54 * (29.5),//Z
                0//Угол относительно поля
        };

        public final ArrayList<double[]> artifactsUnderBlueWallCoord = new ArrayList<double[]>(
                List.of(
                        //                GREEN                                                        PURPLE                                                            PURPLE
                        new double[]{1, -30.401, -133.588, 6.264, Math.toRadians(-90)}, new double[]{2, -30.401, -122.888, 6.264, Math.toRadians(-90)}, new double[]{2, -30.401, -108.188, 6.264, Math.toRadians(-90)},
                        //                PURPLE                                                        GREEN                                                            PURPLE
                        new double[]{2, 29.607, -133.588, 6.264, Math.toRadians(-90)}, new double[]{1, 29.607, -122.888, 6.264, Math.toRadians(-90)}, new double[]{2, 29.607, -108.188, 6.264, Math.toRadians(-90)},
                        //                PURPLE                                                        PURPLE                                                            GREEN
                        new double[]{2, 89.614, -133.588, 6.264, Math.toRadians(-90)}, new double[]{2, 89.614, -122.888, 6.264, Math.toRadians(-90)}, new double[]{1, 89.614, -108.188, 6.264, Math.toRadians(-90)}
                )
        );


        public final ArrayList<double[]> artifactsUnderRedWallCoord = new ArrayList<double[]>(
                List.of(
                        new double[]{1, -30.401, 133.588, 6.264, Math.toRadians(90)}, new double[]{2, -30.401, 122.888, 6.264, Math.toRadians(90)}, new double[]{2, -30.401, 108.188, 6.264, Math.toRadians(90)},
                        new double[]{2, 29.607, 133.588, 6.264, Math.toRadians(90)}, new double[]{1, 29.607, 122.888, 6.264, Math.toRadians(90)}, new double[]{2, 29.607, 108.188, 6.264, Math.toRadians(90)},
                        new double[]{2, 89.614, 133.588, 6.264, Math.toRadians(90)}, new double[]{2, 89.614, 122.888, 6.264, Math.toRadians(90)}, new double[]{1, 89.614, 108.188, 6.264, Math.toRadians(90)}
                )
        );

        public final ZonesDataClass.Builder fireZones = new ZonesDataClass.Builder()
                .createZone(TeamAliance.NEVERMIND, ZonesDataClass.ZoneRole.FIRE, new Dot(0, 0), new Dot(0, 0))
                .createZone(TeamAliance.NEVERMIND, ZonesDataClass.ZoneRole.FIRE, new Dot(0, 0), new Dot(0, 0), new Dot(0, 0))
                .createZone(TeamAliance.NEVERMIND, ZonesDataClass.ZoneRole.FIRE, new Dot(0, 0), new Dot(0, 0), new Dot(0, 0));
        public final ZonesDataClass.Builder baseZones = new ZonesDataClass.Builder()
                .createZone(TeamAliance.BLUE, ZonesDataClass.ZoneRole.PARKING, new Dot(0, 0), new Dot(0, 0))
                .createZone(TeamAliance.RED, ZonesDataClass.ZoneRole.PARKING, new Dot(0, 0), new Dot(0, 0));
    }
}
