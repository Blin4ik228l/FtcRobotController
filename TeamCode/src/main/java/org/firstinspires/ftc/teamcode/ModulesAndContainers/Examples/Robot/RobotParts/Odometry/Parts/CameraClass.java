package org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.Odometry.Parts;

import android.util.Size;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Config.MainFile;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.Odometry.OdometryData;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.Odometry.Parts.MathUtils.Position2D;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Extenders.Extenders2.UpdatableModule;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Extenders.UpdatingModule;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


import java.util.concurrent.TimeUnit;

public class CameraClass extends UpdatableModule {
    private WebcamName webcamName;
    private Position cameraPosition;
    private YawPitchRollAngles cameraOrientation;
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    public GeneralLogic generalLogic;
    public RandomizeStatus randomizeStatus;
    public ElapsedTime updateTime;
    public CameraClass(MainFile mainFile, String searchingDevice)  {
        super(mainFile, searchingDevice);

        try {
            webcamName = hardwareMap.get(WebcamName.class, searchingDevice);
            cameraPosition = new Position(DistanceUnit.CM,0, -16,0, 0);//Позиция камеры относительно координат робота

            cameraOrientation = new YawPitchRollAngles(AngleUnit.RADIANS, Math.toRadians(270) , Math.toRadians(-85), Math.toRadians(0), 0);

            aprilTagProcessor = new AprilTagProcessor.Builder()
                    .setDrawAxes(false)
                    .setDrawCubeProjection(false)
                    .setDrawTagID(true)
                    .setDrawTagOutline(false)
                    .setLensIntrinsics(708.013f, 708.013f, 311.973f, 253.313f)
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
                    .enableLiveView(true)
                    .build();



        }catch (Exception e){
            isInitialized = false;
        }

        generalLogic = GeneralLogic.Check_camera_state;

        randomizeStatus = RandomizeStatus.UnDetected;
        absoluteData = new OdometryData();

        updateTime = new ElapsedTime();
        sayInited();
    }
    public OdometryData absoluteData;
    private ExposureControl exposure;
    private GainControl gain;
    private int index;

    public enum RandomizeStatus{
        Detected,
        UnDetected
    }
    public enum GeneralLogic{
        Check_camera_state,
        Processing,
        Stop
    }
    protected double robotFieldX;
    protected double robotFieldY;
    protected double robotFieldZ;

    protected double robotFieldPitch;
    protected double robotFieldRoll;
    protected double robotFieldYaw;

    protected double rangeToTag;
    protected double cameraElevation;


    protected double ftcFieldYaw;
    protected double ftcFieldPitch;
    protected double ftcFieldRoll;

    protected double ftcFieldX;
    protected double ftcFieldY;
    protected double ftcFieldZ;

    protected double centerX;
    protected double centerY;

    public int id;
    public double cameraBearing;
    public int[] motif = new int[3];
    @Override
    protected void updateExt() {
        switch (generalLogic){
            case Check_camera_state:
                if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {

                    exposure = visionPortal.getCameraControl(ExposureControl.class);
                    exposure.setMode(ExposureControl.Mode.Manual);//Если камера не поддерживает настройку экспозиции
                    exposure.setExposure(5, TimeUnit.MILLISECONDS);//Экспозиция

                    gain = visionPortal.getCameraControl(GainControl.class);

                    //утром 130 - вечером 170
                    gain.setGain(130);//яркость

                    aprilTagProcessor.setDecimation(2.0f);

                    generalLogic = GeneralLogic.Processing;
                }
                break;
            case Processing:
                if (!aprilTagProcessor.getDetections().isEmpty())
                {
                    index = index % aprilTagProcessor.getDetections().size();

                    AprilTagDetection detection = aprilTagProcessor.getDetections().get(index);

                    index++;

                    id = detection.id;

                    setRandomizedArtifactFromId(id);

                    //Если камера смотрит на нужные таги, берём с них позу и повышаем степень уверенности
                    //Уменьшаем её если слишком далеко от тага
                    double decisionMargin;
                    if (id == 20 || id == 24){
                        decisionMargin = detection.decisionMargin;

                        robotFieldX = detection.robotPose.getPosition().x;
                        robotFieldY = detection.robotPose.getPosition().y;
                        robotFieldZ = detection.robotPose.getPosition().z;

                        robotFieldPitch = detection.robotPose.getOrientation().getPitch(AngleUnit.RADIANS);
                        robotFieldRoll  = detection.robotPose.getOrientation().getRoll(AngleUnit.RADIANS);
                        robotFieldYaw   = detection.robotPose.getOrientation().getYaw(AngleUnit.RADIANS);

                        rangeToTag = detection.ftcPose.range;
                        cameraElevation = detection.ftcPose.elevation;
                        cameraBearing   = detection.ftcPose.bearing;

                        ftcFieldYaw = detection.ftcPose.yaw;
                        ftcFieldPitch = detection.ftcPose.pitch;
                        ftcFieldRoll = detection.ftcPose.roll;

                        ftcFieldX = detection.ftcPose.x;
                        ftcFieldY = detection.ftcPose.y;
                        ftcFieldZ = detection.ftcPose.z;

                        centerX = detection.center.x;
                        centerY = detection.center.y;

                        if (rangeToTag > 300){
                            decisionMargin = 0;
                        }
                        absoluteData.setPosition(new Position2D(robotFieldX, robotFieldY, robotFieldYaw));
                    }
                    else{
                        decisionMargin = 0;
                    }
                    absoluteData.setDesisionMarg(decisionMargin);
                }
                else
                {
                    absoluteData.setDesisionMarg(0);
                    index = 0;
                }
                break;

            case Stop:
                visionPortal.stopStreaming();
                break;
        }
}

    public void setRandomizedArtifactFromId(int id) {
        int green = 1;
        int purple = 2;

        if (id != 21 || id != 22 ||id != 23 && randomizeStatus == RandomizeStatus.UnDetected){
            int min = 21;
            int max = 23;
            id = (int)(Math.random() * (max - min + 1)) + min;

            if(id == 21){
                motif = new int[] {green, purple, purple};
            }
            if(id == 22){
                motif = new int[] {purple, green, purple};
            }
            if(id == 23){
                motif = new int[] {purple, purple, green};
            }
        }else {
            if(id == 21){
                motif = new int[] {green, purple, purple};
            }
            if(id == 22){
                motif = new int[] {purple, green, purple};
            }
            if(id == 23){
                motif = new int[] {purple, purple, green};
            }
            randomizeStatus = RandomizeStatus.Detected;
        }
    }

    @Override
    public void showDataExt() {
        sayModuleName();
        telemetry.addData("General logic", generalLogic.toString());
        telemetry.addData("Randomize status", randomizeStatus.toString());
//        telemetry.addData("Camera state", visionPortal.getCameraState().toString());
//        telemetry.addData("onceSeen", onceSeen);
        telemetry.addData("Des", absoluteData.getDesisionMarg());
//        telemetry.addData("Robot Pos", "X:%.2f Y:%.2f Z:%.2f", robotFieldX, robotFieldY, robotFieldZ);
//        telemetry.addData("Robot Angles", "R:%.1f P:%.1f Y:%.1f", robotFieldRoll * RAD, robotFieldPitch * RAD, robotFieldYaw * RAD);
//        telemetry.addData("Camera Angles", "R:%.1f E:%.1f B:%.1f", rangeToTag, cameraElevation * RAD, cameraBearing * RAD);
//        telemetry.addData("FTC Pos", "X:%.2f Y:%.2f Z:%.2f", ftcFieldX, ftcFieldY, ftcFieldZ);
//        telemetry.addData("FTC Angles", "R:%.1f P:%.1f Y:%.1f", ftcFieldRoll * RAD, ftcFieldPitch * RAD, ftcFieldYaw * RAD);
//        telemetry.addData("Center", "X:%.1f Y:%.1f", centerX, centerY);
//        telemetry.addData("Last Pos was taked", updateTime.seconds());
        telemetry.addLine();
    }
}
