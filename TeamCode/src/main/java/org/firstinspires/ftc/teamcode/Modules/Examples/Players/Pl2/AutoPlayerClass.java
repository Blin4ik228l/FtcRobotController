package org.firstinspires.ftc.teamcode.Modules.Examples.Players.Pl2;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Modules.Examples.Players.JoystickActivityClass;
import org.firstinspires.ftc.teamcode.Modules.Examples.Players.PlayerClass;
import org.firstinspires.ftc.teamcode.Robot.RobotClass;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.CollectorParts.CollectorMotors;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.CollectorParts.ColorSensorClass;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.CollectorParts.ServomotorsClass;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DrivetrainParts.CameraClass;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DrivetrainParts.Odometry.OdometryClass;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DrivetrainParts.PositionRobotController;

public class AutoPlayerClass extends PlayerClass{
    public AutoPlayerClass(JoystickActivityClass joystickActivityClass, RobotClass.Collector collector, ElapsedTime elapsedTime, OpMode op) {
        super(joystickActivityClass, op.telemetry);
        this.collector = collector;

        innerTime = elapsedTime;
    }
    public RobotClass.Collector collector;
    public PositionRobotController.VyrState vyrState;
    public OdometryClass.MoveState moveState;
    public OdometryClass.RotateState rotateState;
    public CameraClass.RandomizeStatus randomizeStatus;
    public double range;
    public ElapsedTime innerTime;
    public void setFields(CameraClass.RandomizeStatus randomizeStatus,
            PositionRobotController.VyrState vyrState,
                          OdometryClass.MoveState moveState, OdometryClass.RotateState rotateState, double range){
        this.randomizeStatus = randomizeStatus;
        this.vyrState = vyrState;
        this.moveState = moveState;
        this.rotateState = rotateState;
        this.range = range;
    }
    double targetAngle;
    double targetSpeed, targetRadSpeed;
    int count;
    boolean isLoadEnded;
    public GeneralState generalState = GeneralState.Load;
    public LoadState loadState = LoadState.Prepare_to_load;
    public FireState fireState = FireState.Prepare_to_fire;
    public enum GeneralState {
        Load,
        Fire
    }
    public enum LoadState{
        Idle,
        load_and_check,
        Reverse,
        Rotate_baraban,
        Prepare_to_load,
        Prepare_baraban
    }
    public enum FireState{
        Push_artifact,
        Pusher_back,
        Find_and_turn,
        Idle,
        Prepare_to_fire,
        Check_readiness
    }
    public double theta = 10;

    @Override
    public void execute(){
        double delayToBaraban = BARABAN_DELAY;
        double delayToPusher = PUSHER_DELAY;
        double delayToReverse = REVERSE_DELAY;

        targetAngle = getAngle(range);
        targetSpeed = getSpeed(range, targetAngle);

        targetRadSpeed = targetSpeed / MAX_EXPERIMENTAL_SPEED_IN_METERS * MAX_RAD_SPEED;

//        if(innerTime.seconds() > END_TIME + 5){
//            return;
//        } else if (innerTime.seconds() > END_TIME) {
//            generalState = GeneralState.Fire;
//        }else if(innerTime.seconds() > END_TIME - 1){
//            joystickActivityClass.buttonX = true;
//        }

        if(!joystickActivityClass.buttonX) {
            double barabanPos = BARABAN_CELL0_POS;
            double pusherPos = PUSHER_START_POS;

            collector.servos.setAngle(findNeededPosAngle(targetAngle));

            if(joystickActivityClass.tDpadRightPressed == 1){
                count ++;
                count = Range.clip(count, 0, 3);

                joystickActivityClass.tDpadRightPressed = 0;
            }
            if(joystickActivityClass.tDpadLeftPressed == 1){
                count --;
                count = Range.clip(count, 0, 3);

                joystickActivityClass.tDpadLeftPressed = 0;
            }

            switch (count){
                case 0:
                    barabanPos = 0.0;
                    break;
                case 1:
                    barabanPos = 0.25;
                    break;
                case 2:
                    barabanPos = 0.50;
                    break;
            }

            if(joystickActivityClass.tDpadUpPressed == 0){
                pusherPos = PUSHER_START_POS;
            } else if (joystickActivityClass.tDpadUpPressed == 1) {
                pusherPos = PUSHER_PREFIRE_POS;
            } else if(joystickActivityClass.tDpadUpPressed == 2) {
                pusherPos = PUSHER_ENDING_POS;
            }else {
                joystickActivityClass.tDpadUpPressed = 0;
            }

//            if(collector.servos.runTimePusher.seconds() > delayToPusher){
//                if(pusherPos == PUSHER_ENDING_POS) {
//                    joystickActivityClass.tDpadUpPressed = 1;
//                    pusherPos = PUSHER_PREFIRE_POS;
//                }
//            }
            collector.servos.setPusher(pusherPos);

            if(collector.servos.runTimePusher.seconds() > delayToPusher && collector.servos.curPusherPos <= PUSHER_PREFIRE_POS){
                collector.servos.setBaraban(barabanPos);
            }

            if(isRotateEnded(delayToBaraban)){//Сначала ждём проворота барабана
                if (collector.colorSensorClass.colorState == ColorSensorClass.ColorSensorState.Artifact_Detected ){
                    collector.digitalCellsClass.setColor(collector.colorSensorClass.artifactColor);
                }else {
                    collector.digitalCellsClass.deleteColorFromCell();
                }
            }

            if (joystickActivityClass.bumperRight) {
                collector.motors.setSpeed(5);
            }else {
                collector.motors.setSpeed(0);
            }

            if(joystickActivityClass.bumperLeft && collector.servos.curPusherPos == PUSHER_START_POS){
                collector.motors.onIntake();

            }else if(joystickActivityClass.tLeftBumperPressed != 0 && joystickActivityClass.tLeftBumperPressed % 2 == 0 ) {
                collector.motors.reverseInTake();

                if(collector.motors.runTimeIntake.seconds() > delayToReverse){
                    joystickActivityClass.tLeftBumperPressed = 0;
                }
            }else {
                collector.motors.offIntake();
            }

            if(collector.digitalCellsClass.artifactCount == 3){
                loadState = LoadState.Idle;
            }else {
                loadState = LoadState.Prepare_to_load;
            }
            generalState = GeneralState.Load;

            fireState = FireState.Prepare_to_fire;
        }else{
            joystickActivityClass.tDpadUpPressed = 0;
            joystickActivityClass.tDpadDownPressed = 0;
            joystickActivityClass.bumperRight = false;
            joystickActivityClass.bumperLeft = false;
            joystickActivityClass.tLeftBumperPressed = 0;
            joystickActivityClass.tRightBumperPressed = 0;
            count = 0;

            switch (generalState){
                case Load:
                    switch (loadState) {
                        case Prepare_to_load:
                            collector.motors.offIntake();
                            collector.servos.setPusher(PUSHER_START_POS);

                            if(collector.servos.runTimePusher.seconds() > delayToPusher){
                                loadState = LoadState.Prepare_baraban;
                            }
                            break;

                        case Prepare_baraban:
                            collector.servos.setBaraban(collector.digitalCellsClass.getBarabanPos());

                            if(isRotateEnded(delayToBaraban)){
                                loadState = LoadState.load_and_check;
                            }

                            break;

                        case load_and_check:
                            collector.update();
                            if (collector.colorSensorClass.colorState == ColorSensorClass.ColorSensorState.Artifact_Detected) {
                                collector.motors.offIntake();
//                                collector.servos.setPusher(PUSHER_PREFIRE_POS);

                                collector.digitalCellsClass.setColor(collector.colorSensorClass.artifactColor);

                                if(collector.digitalCellsClass.artifactCount == 3){
                                    loadState = LoadState.Idle;

                                }else {
                                    loadState = LoadState.Reverse;
                                }

                            }else collector.motors.onIntake();

                            break;

                        case Reverse:
                            collector.motors.reverseInTake();

                            if(collector.motors.runTimeIntake.seconds() > delayToReverse){
                                collector.motors.offIntake();
                                loadState = LoadState.Prepare_baraban;
                            }
                            break;

                        case Idle:
                            collector.motors.reverseInTake();
                            if (collector.motors.runTimeIntake.seconds() > delayToReverse) {
                                collector.motors.offIntake();

                                collector.motors.preFireSpeedFlyWheel();
                                collector.servos.setPusher(PUSHER_PREFIRE_POS);

                                generalState = GeneralState.Fire;
                                fireState = FireState.Prepare_to_fire;
                            }

                            break;

                        default:
                            break;
                    }

                    break;

                case Fire:
                    collector.servos.setAngle(findNeededPosAngle(targetAngle));
                    collector.motors.setSpeed(targetRadSpeed);

                    if(collector.servos.angleStates == ServomotorsClass.AngleStates.Unready
                            || collector.motors.flyWheelStates == CollectorMotors.FlyWheelStates.Unready
                            || randomizeStatus == CameraClass.RandomizeStatus.UnDetected
                            || vyrState == PositionRobotController.VyrState.Far_from_it
                            || moveState == OdometryClass.MoveState.High_speed
                            || rotateState == OdometryClass.RotateState.High_speed) {
                        if (fireState == FireState.Push_artifact) return;//Программе дальше нет смысла идти пока не выполнятся условия
                    }

                    switch (fireState) {
                        case Prepare_to_fire:
                            collector.servos.setPusher(PUSHER_PREFIRE_POS);
                            if (collector.servos.runTimePusher.seconds() > delayToPusher) {
                                fireState = FireState.Find_and_turn;
                            }
                            break;

                        case Find_and_turn:
                            if (collector.digitalCellsClass.artifactCount == 0) {
                                fireState = FireState.Idle;
                                break;
                            }

                            // Определяем целевую ячейку (0, 1 или 2)
                            int targetCellIndex = 3 - collector.digitalCellsClass.artifactCount; // 3→0, 2→1, 1→2
                            int targetColor = collector.digitalCellsClass.getRandomizedArtifact()[targetCellIndex];

                            double targetPos = collector.digitalCellsClass.findNeededArtifactPos(targetColor);

                            collector.servos.setBaraban(targetPos);
                            if(isRotateEnded(delayToBaraban) ){
                                fireState = FireState.Push_artifact;
                            }
                            break;

                        case Push_artifact:
                            collector.servos.setPusher(PUSHER_ENDING_POS);

                            if (collector.servos.runTimePusher.seconds() > delayToPusher) {
                                fireState = FireState.Pusher_back;
                            }
                            break;

                        case Pusher_back:
                            collector.servos.setPusher(PUSHER_PREFIRE_POS);

                            if (collector.servos.runTimePusher.seconds() > delayToPusher) {
                                if (collector.colorSensorClass.colorState == ColorSensorClass.ColorSensorState.No_Artifact_Detected) {
                                    collector.digitalCellsClass.deleteColorFromCell();
                                    fireState = FireState.Find_and_turn;
                                } else {
                                    fireState = FireState.Push_artifact;
                                }
                            }
                            break;

                        case Idle:
                            collector.motors.offFLyWheel();
                            collector.servos.setPusher(PUSHER_START_POS);

                            generalState = GeneralState.Load;
                            loadState = LoadState.Prepare_baraban;
                            break;

                        default:
                            break;
                    }

                    break;
                default:
                    throw new IllegalStateException("Unexpected value: " + generalState);
            }
        }
    }
    public boolean isRotateEnded(double delayToBaraban){
        return collector.servos.runTimeBaraban.seconds() > delayToBaraban || collector.buttonClass.getState();
    }
    public double findNeededPosAngle(double targAngle){
        double rampAngle = Range.clip(90 - Math.toDegrees(targAngle), MIN_ANGLE, 65);

        return (65 - rampAngle) * (185 / 23) / 270;
    }
    double getAngle(double range){
        return Math.atan(Math.tan(Math.toRadians(theta)) + 2 * (80) / range);
    }
    double getSpeed(double range, double angle){
        return Math.sqrt(981 * range / ((Math.tan(Math.toRadians(theta)) + Math.tan(angle)) * Math.pow(Math.cos(angle), 2))) / 100;
    }
    @Override
    public void showData(){
        telemetry.addLine("===AUTO PLAYER===");
        telemetry.addData("Automatic state", generalState.toString());

        if(generalState == GeneralState.Load){
            telemetry.addData("Load state", loadState.toString());
        }else telemetry.addData("Fire state", fireState.toString());

        telemetry.addData("Angle", collector.servos.angleStates.toString());
        telemetry.addData("FlyWheel", collector.motors.flyWheelStates.toString());
        telemetry.addData("Randomize", randomizeStatus.toString());
        telemetry.addData("VyrState", vyrState.toString());
        telemetry.addData("MoveState", moveState.toString());
        telemetry.addData("RotateState", rotateState.toString());
        telemetry.addLine();
    }

}
