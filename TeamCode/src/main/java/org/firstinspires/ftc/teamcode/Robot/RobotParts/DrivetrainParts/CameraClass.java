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


import java.util.concurrent.TimeUnit;

public class CameraClass extends UpdatableModule {
    public CameraClass(OpMode op, TeamColor teamColor)  {
        super(op.telemetry);

        this.teamColor = teamColor;

        webcamName = op.hardwareMap.get(WebcamName.class, "Webcam 1");

        cameraPosition = new Position(DistanceUnit.CM,-9 ,-15,26.086, 0);//Позиция камеры относительно координат робота
        cameraOrientation = new YawPitchRollAngles(AngleUnit.RADIANS, Math.toRadians(0), Math.toRadians(87), Math.toRadians(180), 0);//Насколько камера повёрнута относительно неё же

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

        while (true){
            if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
                isCameraStreaming = true;
                break;}
        }

        exposure = visionPortal.getCameraControl(ExposureControl.class);
        exposure.setMode(ExposureControl.Mode.Manual);//Если камера не поддерживает настройку экспозиции
        exposure.setExposure(5, TimeUnit.MILLISECONDS);//Экспозиция

        gain = visionPortal.getCameraControl(GainControl.class);
        gain.setGain(190);//яркость

        aprilTagProcessor.setDecimation(2);

        position2D = null;

        randomizeStatus = RandomizeStatus.UnDetected;

        lastPosWasTaked = new ElapsedTime();
        tagState = TagState.noDetected;

        telemetry.addLine("Camera Inited");
    }
    private final Position cameraPosition;
    private final YawPitchRollAngles cameraOrientation;
    private final AprilTagProcessor aprilTagProcessor;
    private final VisionPortal visionPortal;
    public AprilTagDetection detection;
    private final WebcamName webcamName;
    private final ExposureControl exposure;
    private final GainControl gain;
    public final TeamColor teamColor;
    private final int green = 1;
    private final int purple = 2;
    private int id;
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
    public double cameraFov = Math.toRadians(60);
    public Position2D robotPos;
    public double robotVel;
    public double robotHeadVel;
    public ElapsedTime lastPosWasTaked;
    public TagState tagState;
    public Position2D position2D;
    public RandomizeStatus randomizeStatus;
    public enum TagState {
        noDetected,
        hasDetected
    }
    public enum RandomizeStatus{
        Detected,
        UnDetected
    }
    public GeneralLogic generalLogic;
    public CameraLogic cameraLogic;
    public enum CameraLogic{
        Check_condition,
        Get_pos
    }
    public enum GeneralLogic{
        Start_game,
        Button_play_pressed,
        Need_movement_ASAP,
        Mid_game
    }
    public void setFields(Position2D robotPos, double robotHeadVel, double robotCurVel){
        this.robotPos = robotPos;
        this.robotHeadVel = robotHeadVel;
        robotVel = robotCurVel;
    }
    public double distanceWeight;
    public double angleWeight;
    public double qualityWeight;
    public double combinedWeight;

    public int atemptTryies = 0;
    @Override
    public void update(){
        switch (generalLogic){
            case Start_game:
                switch (cameraLogic){
                    case Check_condition:
                        if(!aprilTagProcessor.getDetections().isEmpty()){
                            cameraLogic = CameraLogic.Get_pos;
                        }
                        break;

                    case Get_pos:
                        for(AprilTagDetection detection1 : aprilTagProcessor.getDetections()){
                            id = detection1.id;

                            if((id == 20 || id == 24) ){
                                robotFieldX = detection1.robotPose.getPosition().x;
                                robotFieldY = detection1.robotPose.getPosition().y;
                                robotFieldZ = detection1.robotPose.getPosition().z;

                                robotFieldPitch = detection1.robotPose.getOrientation().getPitch(AngleUnit.RADIANS);
                                robotFieldRoll  = detection1.robotPose.getOrientation().getRoll(AngleUnit.RADIANS);
                                robotFieldYaw   = detection1.robotPose.getOrientation().getYaw(AngleUnit.RADIANS);

                                robotRangeToTag = detection1.ftcPose.range;
                                cameraElevation = detection1.ftcPose.elevation;
                                cameraBearing   = detection1.ftcPose.bearing;

                                position2D = new Position2D(robotFieldX, robotFieldY, robotFieldYaw);

                                tagState = TagState.hasDetected;
                            }else {
                                tagState = TagState.noDetected;
                            }
                        }

                        cameraLogic = CameraLogic.Check_condition;
                        break;
                }
                break;

            case Button_play_pressed:
                switch (cameraLogic){
                    case Check_condition:
                        if(!aprilTagProcessor.getDetections().isEmpty()){
                            cameraLogic = CameraLogic.Get_pos;
                        }
                    break;

                    case Get_pos:
                        atemptTryies ++;

                        if(id == 21 || id == 22 || id == 23){
                            setRandomizedArtifactFromId(id);
                            randomizeStatus = RandomizeStatus.Detected;
                        }
                        if(randomizeStatus == RandomizeStatus.Detected){
                            cameraLogic = CameraLogic.Check_condition;
                            generalLogic = GeneralLogic.Mid_game;

                        }else if(atemptTryies == 2){
                            generalLogic = GeneralLogic.Need_movement_ASAP;
                        }else {
                            cameraLogic = CameraLogic.Check_condition;
                        }
                        break;

                }
                break;

            case Need_movement_ASAP:
                break;

            case Mid_game:
                switch (cameraLogic){
                    case Check_condition:
                        if(isTagShouldBeInFov() && lastPosWasTaked.seconds() > 1){
                            if(!aprilTagProcessor.getDetections().isEmpty()){
                                cameraLogic = CameraLogic.Get_pos;
                            }else {
                                tagState = TagState.noDetected;
                            }
                        }else {
                            tagState = TagState.noDetected;
                        }
                        break;

                    case Get_pos:
                        for(AprilTagDetection detection1 : aprilTagProcessor.getDetections()){
                            id = detection1.id;

                            if((id == 20 || id == 24) ){
                                robotFieldX = detection1.robotPose.getPosition().x;
                                robotFieldY = detection1.robotPose.getPosition().y;
                                robotFieldZ = detection1.robotPose.getPosition().z;

                                robotFieldPitch = detection1.robotPose.getOrientation().getPitch(AngleUnit.RADIANS);
                                robotFieldRoll  = detection1.robotPose.getOrientation().getRoll(AngleUnit.RADIANS);
                                robotFieldYaw   = detection1.robotPose.getOrientation().getYaw(AngleUnit.RADIANS);

                                robotRangeToTag = detection1.ftcPose.range;
                                cameraElevation = detection1.ftcPose.elevation;
                                cameraBearing   = detection1.ftcPose.bearing;

                                position2D = new Position2D(robotFieldX, robotFieldY, robotFieldYaw);

                                distanceWeight = getDistanceBasedWeight(detection.ftcPose.range);
                                angleWeight = getAngleBasedWeight(
                                        detection.ftcPose.bearing,
                                        detection.ftcPose.elevation
                                );
                                qualityWeight = getQualityBasedWeight(detection);

                                // Комбинируем все факторы (можно с разными весами)
                                combinedWeight = distanceWeight * 0.5
                                        + angleWeight * 0.3
                                        + qualityWeight * 0.2;

                                tagState = TagState.hasDetected;
                                lastPosWasTaked.reset();
                            }
                        }
                        cameraLogic = CameraLogic.Check_condition;
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
    public Position2D getPos(){
        if(tagState == TagState.noDetected) return null;

        return new Position2D(robotFieldX, robotFieldY, robotFieldYaw);
    }
    public void setRandomizedArtifactFromId(int id) {
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

    public boolean isTagShouldBeInFov(){
        double leftBorder = robotPos.getHeading() + (Math.signum(robotPos.getHeading()) * cameraFov / 4.0);
        double rightBorder = robotPos.getHeading() - (Math.signum(robotPos.getHeading()) * cameraFov / 4.0);

        return !((Math.toRadians(44) < leftBorder && rightBorder < Math.toRadians(44) && robotPos.getY() > 0)
                        ||
                (Math.toRadians(136) < (leftBorder) && (rightBorder) < Math.toRadians(136)) && robotPos.getY() < 0);
    }
    public boolean isRobotHaveMinVel(){
       return Math.abs(robotHeadVel) <= Math.toRadians(30);
    }
    public boolean isRobotHaveMinRange(){
        return robotRangeToTag < 200 && robotRangeToTag > 40;
    }

    public void showData(){
        telemetry.addLine("=== CAMERA ===");

        telemetry.addData("Tags Found", 0);
        telemetry.addData("Streaming", isCameraStreaming);
        telemetry.addData("ftc yaw", ftcYaw * rad);
        telemetry.addData("Robot Pos", "X:%.2f Y:%.2f Z:%.2f", robotFieldX, robotFieldY, robotFieldZ);
        telemetry.addData("Robot Angles", "R:%.1f P:%.1f Y:%.1f", robotFieldRoll * rad, robotFieldPitch * rad, robotFieldYaw * rad);
        telemetry.addData("Camera Angles", "R:%.1f E:%.1f B:%.1f", robotRangeToTag, cameraElevation * rad, cameraBearing * rad);
        telemetry.addData("Camera state", tagState.toString());
        telemetry.addData("Robot have min vel", isRobotHaveMinVel());
        telemetry.addData("Robot have min range", isRobotHaveMinRange());
        telemetry.addData("Camera have needed fov", isTagShouldBeInFov());
        telemetry.addData("Last Pos was taked", lastPosWasTaked.seconds());
        telemetry.addLine();
    }
}
