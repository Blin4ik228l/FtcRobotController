package org.firstinspires.ftc.teamcode.Robot.RobotParts.DrivetrainParts;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Modules.Types.UpdatableModule;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DrivetrainParts.Odometry.Parts.MathUtils.Position2D;
import org.firstinspires.ftc.teamcode.Robot.TeamColor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

public class CameraClass extends UpdatableModule {
    public CameraClass(OpMode op, TeamColor teamColor)  {
        super(op.telemetry);
        this.teamColor = teamColor;

        webcamName = op.hardwareMap.get(WebcamName.class, "Webcam 1");

        cameraPosition = new Position(DistanceUnit.CM,-13 ,8,0, 0);//Позиция камеры относительно координат робота
        cameraOrientation = new YawPitchRollAngles(AngleUnit.RADIANS, Math.toRadians(90) * 1, Math.toRadians(-80) * 1, Math.toRadians(0) * 1, 0);//Насколько камера повёрнута относительно неё же

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(false)
                .setDrawCubeProjection(false)
                .setDrawTagID(true)
                .setDrawTagOutline(false)
                .setLensIntrinsics(708.013f, 708.013f, 311.973, 253.313f)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                .setOutputUnits(DistanceUnit.CM, AngleUnit.RADIANS)
                .setCameraPose(cameraPosition, cameraOrientation)
                .build();

        visionPortal = new VisionPortal.Builder()
                .addProcessor(aprilTagProcessor)
                .setCamera(webcamName)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();

        lastRecordedPosition2D = null;

        lastPosWasTaked = new ElapsedTime();

        generalLogic = GeneralLogic.Check_camera_state;
        cameraLogic = CameraLogic.Check_condition;

        randomizeStatus = RandomizeStatus.UnDetected;
        tagState = TagState.UnDetected;

        telemetry.addLine("Camera Inited");
    }
    private final Position cameraPosition;
    private final YawPitchRollAngles cameraOrientation;
    private final AprilTagProcessor aprilTagProcessor;
    private final VisionPortal visionPortal;
    private final WebcamName webcamName;
    private ExposureControl exposure;
    private GainControl gain;
    public final TeamColor teamColor;
    public double robotFieldX;
    public double robotFieldY;
    public double robotFieldZ;
    public double tagXFromRobot;
    public double tagYFromRobot;
    public double tagZFromRobot;
    public double robotFieldPitch;
    public double robotFieldRoll;
    public double robotFieldYaw;
    public double ftcYaw;
    public double robotRangeToTag;
    public double cameraElevation;
    public double cameraBearing;

    private boolean isObeliskWasSeen = false;
    private boolean isPosWasTakenFirstly = false;
    private boolean isCameraStreaming;
    public double rad = 180 / Math.PI;

    public Position2D robotPos;
    public double robotVel;
    public double robotHeadVel;

    public ElapsedTime lastPosWasTaked;

    private Position2D lastRecordedPosition2D;


    public double distanceWeight;
    public double angleWeight;
    public double qualityWeight;
    public double combinedWeight;
    public int index;
    public int id;
    public ArrayList <AprilTagDetection> lastRecordedDetection = new ArrayList<>();
    public enum TagState {
        Detected,
        UnDetected
    }
    public enum RandomizeStatus{
        Detected,
        UnDetected
    }

    public GeneralLogic generalLogic;
    public CameraLogic cameraLogic;
    public RandomizeStatus randomizeStatus;
    public TagState tagState;

    public enum CameraLogic{
        Check_condition,
        Get_pos
    }
    public enum GeneralLogic{
        Check_camera_state,
        Check_obelisk_and_pos,
        Check_only_pos
    }
    public void setFields(Position2D robotPos, double robotHeadVel, double robotCurVel){
        this.robotPos = robotPos;
        this.robotHeadVel = robotHeadVel;
        robotVel = robotCurVel;
    }

    @Override
    public void update(){
        switch (generalLogic){
            case Check_camera_state:

                if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
                    exposure = visionPortal.getCameraControl(ExposureControl.class);
                    exposure.setMode(ExposureControl.Mode.Manual);//Если камера не поддерживает настройку экспозиции
                    exposure.setExposure(5, TimeUnit.MILLISECONDS);//Экспозиция

                    gain = visionPortal.getCameraControl(GainControl.class);
                    gain.setGain(190);//яркость

                    aprilTagProcessor.setDecimation(2);

                    isCameraStreaming = true;

                    generalLogic = GeneralLogic.Check_obelisk_and_pos;
                    cameraLogic = CameraLogic.Check_condition;
                }

                break;
            case Check_obelisk_and_pos:

                switch (cameraLogic){

                    case Check_condition:
                        if(!aprilTagProcessor.getDetections().isEmpty() && lastPosWasTaked.seconds() > 0.3){
                            lastRecordedDetection = aprilTagProcessor.getDetections();
                            cameraLogic = CameraLogic.Get_pos;
                        }else {
                            lastRecordedDetection = new ArrayList<>();
                            randomizeStatus = RandomizeStatus.UnDetected;
                            tagState = TagState.UnDetected;
                        }
                        break;

                    case Get_pos:
                        AprilTagDetection detection = lastRecordedDetection.get(index);
                        id = detection.id;

                        if(id == 21 || id == 22 || id == 23){
                            setRandomizedArtifactFromId(id);
                            randomizeStatus = RandomizeStatus.Detected;
                        }

                        if((id == 20 || id == 24) ){
                            robotFieldX = detection.robotPose.getPosition().x;
                            robotFieldY = detection.robotPose.getPosition().y;
                            robotFieldZ = detection.robotPose.getPosition().z;

                            robotFieldPitch = detection.robotPose.getOrientation().getPitch(AngleUnit.RADIANS);
                            robotFieldRoll  = detection.robotPose.getOrientation().getRoll(AngleUnit.RADIANS);
                            robotFieldYaw   = detection.robotPose.getOrientation().getYaw(AngleUnit.RADIANS);

                            robotRangeToTag = detection.ftcPose.range;
                            cameraElevation = detection.ftcPose.elevation;
                            cameraBearing   = detection.ftcPose.bearing;

                            lastRecordedPosition2D = new Position2D(robotFieldX, robotFieldY, robotFieldYaw);

                            tagState = TagState.Detected;
                        }

                        index++;

                        if(index == lastRecordedDetection.size()){
                            if(randomizeStatus == RandomizeStatus.Detected) generalLogic = GeneralLogic.Check_only_pos;
                            cameraLogic = CameraLogic.Check_condition;

                            index = 0;
                            lastPosWasTaked.reset();
                        }
                        break;

                }

                break;

            case Check_only_pos:
                switch (cameraLogic){

                    case Check_condition:
                        if(!aprilTagProcessor.getDetections().isEmpty() && lastPosWasTaked.seconds() > 1){
                            lastRecordedDetection = aprilTagProcessor.getDetections();
                            cameraLogic = CameraLogic.Get_pos;
                        }else {
                            lastRecordedDetection = new ArrayList<>();
                            tagState = TagState.UnDetected;
                        }
                        break;

                    case Get_pos:
                        AprilTagDetection detection = lastRecordedDetection.get(index);
                        id = detection.id;

                        if((id == 20 || id == 24) ){
                            robotFieldX = detection.robotPose.getPosition().x;
                            robotFieldY = detection.robotPose.getPosition().y;
                            robotFieldZ = detection.robotPose.getPosition().z;

                            robotFieldPitch = detection.robotPose.getOrientation().getPitch(AngleUnit.RADIANS);
                            robotFieldRoll  = detection.robotPose.getOrientation().getRoll(AngleUnit.RADIANS);
                            robotFieldYaw   = detection.robotPose.getOrientation().getYaw(AngleUnit.RADIANS);

                            robotRangeToTag = detection.ftcPose.range;
                            cameraElevation = detection.ftcPose.elevation;
                            cameraBearing   = detection.ftcPose.bearing;

                            lastRecordedPosition2D = new Position2D(robotFieldX, robotFieldY, robotFieldYaw);

                            distanceWeight = getDistanceBasedWeight(detection.ftcPose.range);
                            angleWeight = getAngleBasedWeight(
                                    detection.ftcPose.bearing,
                                    detection.ftcPose.elevation
                            );
                            qualityWeight = getQualityBasedWeight(detection);

                            // Комбинируем все факторы (можно с разными весами)
                            combinedWeight = distanceWeight * 0.5 + angleWeight * 0.3 + qualityWeight * 0.2;

                            tagState = TagState.Detected;
                        }

                        index++;

                        if(index == lastRecordedDetection.size()){
                            cameraLogic = CameraLogic.Check_condition;
                            index = 0;

                            lastPosWasTaked.reset();
                        }
                        break;

                }
                break;
        }
    }

    private double getDistanceBasedWeight(double range) {
        // Чем ближе — тем выше доверие
        double maxRange = 355;
        double minRange = 100;

        if (range < minRange) return 1.0; // Максимальное доверие вблизи
        if (range > maxRange) return 0.1; // Минимальное доверие далеко

        // Линейное уменьшение
        return 1.0 - (range - minRange) / (maxRange - minRange) * 0.9;

        // Или квадратичное (быстрее падает)
        // double t = (range - minRange) / (maxRange - minRange);
        // return 1.0 - t * t * 0.9;
    }

    private double getAngleBasedWeight(double bearing, double elevation) {
        // Прямо по центру = максимум доверия
        // С краю = меньше доверия

        double maxBearing = Math.toRadians(30); // ±30°
        double maxElevation = Math.toRadians(20);

        double bearingScore = 1.0 - Math.abs(bearing) / maxBearing;
        double elevationScore = 1.0 - Math.abs(elevation) / maxElevation;

        return Math.max(0, bearingScore * elevationScore);
    }
    private double getQualityBasedWeight(AprilTagDetection detection) {
        if (detection.decisionMargin < 20) return 0.3; // Низкое качество
        if (detection.decisionMargin < 50) return 0.7; // Среднее
        return 1.0; // Высокое качество
    }
    public Position2D getLastRecordedPosition2D(){
        return lastRecordedPosition2D;
    }
    public void setRandomizedArtifactFromId(int id) {
        int green = 1;
        int purple = 2;

        if(id == 21){
            teamColor.setRandomizedArtifact(new int[] {green, purple, purple});
            isObeliskWasSeen = true;
        }
        if(id == 22){
            teamColor.setRandomizedArtifact(new int[] {purple, green, purple});
            isObeliskWasSeen = true;
        }
        if(id == 23){
            teamColor.setRandomizedArtifact(new int[] {purple, purple, green});
            isObeliskWasSeen = true;
        }
    }

    public void showData(){
        telemetry.addLine("=== CAMERA ===");
        telemetry.addData("Camera state", visionPortal.getCameraState().toString());
        telemetry.addData("General logic", generalLogic.toString());
        telemetry.addData("Camera logic", cameraLogic.toString());
        telemetry.addData("Tag status", tagState.toString());
        telemetry.addData("Randomize status", randomizeStatus.toString());
        telemetry.addData("Tags Found", lastRecordedDetection.size());
        telemetry.addData("Last Pos was taked", lastPosWasTaked.seconds());
        telemetry.addData("Robot Pos", "X:%.2f Y:%.2f Z:%.2f", robotFieldX, robotFieldY, robotFieldZ);
        telemetry.addData("Robot Angles", "R:%.1f P:%.1f Y:%.1f", robotFieldRoll * rad, robotFieldPitch * rad, robotFieldYaw * rad);
        telemetry.addData("Camera Angles", "R:%.1f E:%.1f B:%.1f", robotRangeToTag, cameraElevation * rad, cameraBearing * rad);
        telemetry.addLine();
    }
}
