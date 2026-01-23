package org.firstinspires.ftc.teamcode.Robot.RobotParts.DriveTrain.DrivetrainParts;

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
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DriveTrain.DrivetrainParts.Odometry.Parts.MathUtils.Position2D;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

public class CameraClass extends UpdatableModule {
    private final WebcamName webcamName;
    private final Position cameraPosition;
    private final YawPitchRollAngles cameraOrientation;
    private final AprilTagProcessor aprilTagProcessor;
    private final VisionPortal visionPortal;
    public GeneralLogic generalLogic;
    public RandomizeStatus randomizeStatus;
    public TagState tagState;
    private Position2D lastRecordedPosition2D;
    public ElapsedTime updateTime;
    public boolean onceSeen;
    public CameraClass(OpMode op)  {
        super(op);

        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

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
                .build();

        lastRecordedPosition2D = null;
        randomizedArtifacts = new int[3];
        lastRecordedDetection = new ArrayList<>();

        generalLogic = GeneralLogic.Check_camera_state;

        randomizeStatus = RandomizeStatus.UnDetected;
        tagState = TagState.UnDetected;

        updateTime = new ElapsedTime();

        telemetry.addLine("Camera Inited");
    }
    private ExposureControl exposure;
    private GainControl gain;
    private double robotFieldX, robotFieldY, robotFieldZ;
    private double robotFieldPitch, robotFieldRoll, robotFieldYaw;
    public double robotRangeToTag, cameraElevation, cameraBearing;
    private double desicionMargin;
    private int index;
    public int id;
    private int[] randomizedArtifacts;
    public ArrayList <AprilTagDetection> lastRecordedDetection;
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

    public int[] getRandomizedArtifacts() {
        return randomizedArtifacts;
    }

    @Override
    public void update(){
        switch (generalLogic){
            case Check_camera_state:
                if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
                    exposure = visionPortal.getCameraControl(ExposureControl.class);
                    exposure.setMode(ExposureControl.Mode.Manual);//Если камера не поддерживает настройку экспозиции
                    exposure.setExposure(10, TimeUnit.MILLISECONDS);//Экспозиция

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
                    desicionMargin = detection.decisionMargin;

                    setRandomizedArtifactFromId(id);

                    tagState = TagState.UnDetected;

                    if ((id == 20 || id == 24) && desicionMargin > 30 && updateTime.seconds() > 0.1){
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

                        onceSeen = true;
                        tagState = TagState.Detected;

                        updateTime.reset();
                    }
                }
                else
                {
                    index = 0;
                    id = 0;
                    tagState = TagState.UnDetected;
                }
                break;

            case Stop:
                visionPortal.stopStreaming();
                break;
        }
    }
    public void reset(){
        index = 0;
        id = 0;
        tagState = TagState.UnDetected;
    }
    public Position2D getLastRecordedPosition2D(){
        return lastRecordedPosition2D;
    }
    public void setRandomizedArtifactFromId(int id) {
        int green = 1;
        int purple = 2;

//        if(id != 21 || id != 22 || id != 23){
//            int min = 21;
//            int max = 23;
//            id = (int)(Math.random() * (max - min + 1)) + min;
//        }

        if(id == 21){
            randomizedArtifacts = new int[] {green, purple, purple};
            randomizeStatus = RandomizeStatus.Detected;
        }
        if(id == 22){
            randomizedArtifacts = new int[] {purple, green, purple};
            randomizeStatus = RandomizeStatus.Detected;
        }
        if(id == 23){
            randomizedArtifacts = new int[] {purple, purple, green};
            randomizeStatus = RandomizeStatus.Detected;
        }
    }

    public void showData(){
        telemetry.addLine("===CAMERA===");
//        telemetry.addData("General logic", generalLogic.toString());
//        telemetry.addData("Tag status", tagState.toString());
        telemetry.addData("Randomize status", randomizeStatus.toString());
        telemetry.addData("Camera state", visionPortal.getCameraState().toString());
        telemetry.addData("onceSeen", onceSeen);
//        telemetry.addData("index/id", "%s %s",index, id);
//        telemetry.addData("des/yaw", "%s",desicionMargin);
//        telemetry.addData("Robot Pos", "X:%.2f Y:%.2f Z:%.2f", robotFieldX, robotFieldY, robotFieldZ);
//        telemetry.addData("Robot Angles", "R:%.1f P:%.1f Y:%.1f", robotFieldRoll * RAD, robotFieldPitch * RAD, robotFieldYaw * RAD);
//        telemetry.addData("Camera Angles", "R:%.1f E:%.1f B:%.1f", robotRangeToTag, cameraElevation * RAD, cameraBearing * RAD);
//        telemetry.addData("Last Pos was taked", updateTime.seconds());
        telemetry.addLine();
    }
}
