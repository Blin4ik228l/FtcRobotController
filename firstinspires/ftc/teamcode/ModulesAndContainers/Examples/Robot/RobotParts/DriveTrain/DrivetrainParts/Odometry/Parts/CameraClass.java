package org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.DriveTrain.DrivetrainParts.Odometry.Parts;

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
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Extenders.UpdatableModule;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.DriveTrain.DrivetrainParts.Odometry.OdometryData;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.DriveTrain.DrivetrainParts.Odometry.Parts.MathUtils.Position2D;
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
    public TagState tagState;
    private Position2D lastRecordedPosition2D;
    public ElapsedTime updateTime;
    public boolean onceSeen;
    public CameraClass(OpMode op)  {
        super(op);

        try {
            webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        } catch (Exception e) {
            isInizialized = false;
            return;
        }

        cameraPosition = new Position(DistanceUnit.CM,0, -16,0, 0);//Позиция камеры относительно координат робота

//        cameraOrientation = new YawPitchRollAngles(AngleUnit.RADIANS, Math.toRadians(90) * 1, Math.toRadians(-80) * 1, Math.toRadians(0) * 1, 0);
        //Насколько камера повёрнута относительно неё же

        cameraOrientation = new YawPitchRollAngles(AngleUnit.RADIANS, Math.toRadians(180) , Math.toRadians(-85), Math.toRadians(0), 0);

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

        generalLogic = GeneralLogic.Check_camera_state;

        randomizeStatus = RandomizeStatus.UnDetected;
        tagState = TagState.UnDetected;

        dataO = new OdometryData();

        updateTime = new ElapsedTime();

        telemetry.addLine("Camera Inited");
    }
    public OdometryData dataO;
    private ExposureControl exposure;
    private GainControl gain;
    private int index;
    public enum TagState {
        Detected,
        UnDetected
    }
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
    protected double cameraBearing;

    protected double ftcFieldYaw;
    protected double ftcFieldPitch;
    protected double ftcFieldRoll;

    protected double ftcFieldX;
    protected double ftcFieldY;
    protected double ftcFieldZ;

    protected double centerX;
    protected double centerY;

    protected int id;
    @Override
    public void update(){
        if (!isInizialized) return;

        switch (generalLogic){
            case Check_camera_state:
                if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
                    exposure = visionPortal.getCameraControl(ExposureControl.class);
                    exposure.setMode(ExposureControl.Mode.Manual);//Если камера не поддерживает настройку экспозиции
                    exposure.setExposure(20, TimeUnit.MILLISECONDS);//Экспозиция

                    gain = visionPortal.getCameraControl(GainControl.class);

                    //TODO яркость уменьшить
                    gain.setGain(190);//яркость

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
                        dataO.setRobotPosition(new Position2D(robotFieldX, robotFieldY, robotFieldYaw));
                    }
                    else{
                        decisionMargin = 0;
                    }
                    dataO.setDesisionMarg(decisionMargin);
                }
                else
                {
                    dataO.setDesisionMarg(0);
                    index = 0;
                    tagState = TagState.UnDetected;
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

        if (id != 21 || id != 22 ||id != 23){
            int min = 21;
            int max = 23;
            id = (int)(Math.random() * (max - min + 1)) + min;
        }else {
            randomizeStatus = RandomizeStatus.Detected;
        }

        if(randomizeStatus == RandomizeStatus.UnDetected){
            if(id == 21){
                setRandomizedArtifacts(new int[] {green, purple, purple});
            }
            if(id == 22){
                dataT.setRandomizedArtifacts(new int[] {purple, green, purple});
            }
            if(id == 23){
                dataT.setRandomizedArtifacts(new int[] {purple, purple, green});
            }
        }
    }
    @Override
    public void showData(){
        telemetry.addLine("===CAMERA===");
        if (isInizialized) {
            telemetry.addLine();
            telemetry.addData("General logic", generalLogic.toString());
            telemetry.addData("Tag status", tagState.toString());
            telemetry.addData("Randomize status", randomizeStatus.toString());
            telemetry.addData("Camera state", visionPortal.getCameraState().toString());
            telemetry.addData("onceSeen", onceSeen);
            telemetry.addData("Robot Pos", "X:%.2f Y:%.2f Z:%.2f", robotFieldX, robotFieldY, robotFieldZ);
            telemetry.addData("Robot Angles", "R:%.1f P:%.1f Y:%.1f", robotFieldRoll * RAD, robotFieldPitch * RAD, robotFieldYaw * RAD);
            telemetry.addData("Camera Angles", "R:%.1f E:%.1f B:%.1f", rangeToTag, cameraElevation * RAD, cameraBearing * RAD);
            telemetry.addData("FTC Pos", "X:%.2f Y:%.2f Z:%.2f", ftcFieldX, ftcFieldY, ftcFieldZ);
            telemetry.addData("FTC Angles", "R:%.1f P:%.1f Y:%.1f", ftcFieldRoll * RAD, ftcFieldPitch * RAD, ftcFieldYaw * RAD);
            telemetry.addData("Center", "X:%.1f Y:%.1f", centerX, centerY);
            telemetry.addData("Last Pos was taked", updateTime.seconds());
        }else{
            telemetry.addLine("DEVICE NOT FOUND");
        }

        telemetry.addLine();
    }
}
