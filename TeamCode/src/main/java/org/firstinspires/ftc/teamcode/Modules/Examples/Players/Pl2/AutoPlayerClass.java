package org.firstinspires.ftc.teamcode.Modules.Examples.Players.Pl2;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Modules.Joysticks.JoystickActivityClass;
import org.firstinspires.ftc.teamcode.Modules.Examples.Players.PlayerClass;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.Collector.Collector;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.Collector.CollectorParts.CollectorMotors;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.Collector.CollectorParts.ColorSensorClass;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DriveTrain.DrivetrainParts.CameraClass;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DriveTrain.DrivetrainParts.Odometry.OdometryClass;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DriveTrain.DrivetrainParts.PositionRobotController;

public class AutoPlayerClass extends PlayerClass{
    public AutoPlayerClass(JoystickActivityClass joystickActivityClass, Collector collector, ElapsedTime elapsedTime, OpMode op) {
        super(joystickActivityClass, op.telemetry);
        this.collector = collector;

        innerTime = elapsedTime;

        vyrState = PositionRobotController.VyrState.Far_from_it;
        moveState = OdometryClass.MoveState.High_speed;
        rotateState = OdometryClass.RotateState.High_speed;
        randomizeStatus = CameraClass.RandomizeStatus.UnDetected;

        generalState = GeneralState.LoadLogic;
        loadState = LoadState.Prepare;
        fireState = FireState.Prepare;
        anotherStates = AnotherStates.Push;
    }
    private Collector collector;
    private PositionRobotController.VyrState vyrState;
    private OdometryClass.MoveState moveState;
    private OdometryClass.RotateState rotateState;
    private CameraClass.RandomizeStatus randomizeStatus;
    private double range;
    private ElapsedTime innerTime;
    private double targetAngle, targetServoPos;
    private double targetSpeed, targetRadSpeed;
    private int cellNum, attempts;
    private double theta = 30;
    private int flag = 0;
    private GeneralState generalState;
    private LoadState loadState;
    private FireState fireState;
    private AnotherStates anotherStates;
    public enum GeneralState {
        LoadLogic,
        FireLogic
    }
    public enum LoadState{
        Prepare,
        Find,
        Load,
        Idle,
    }
    public enum FireState{
        Prepare,
        Find,
        Fire,
        Idle
    }
    public enum AnotherStates{
        Push,
        Back
    }
    public void setFields(CameraClass.RandomizeStatus randomizeStatus,
                          PositionRobotController.VyrState vyrState,
                          OdometryClass.MoveState moveState, OdometryClass.RotateState rotateState, double range){
        this.randomizeStatus = randomizeStatus;
        this.vyrState = vyrState;
        this.moveState = moveState;
        this.rotateState = rotateState;
        this.range = range;
    }

    @Override
    public void execute(){
        calcAngle();
        calcAngleToPos();
        calcSpeed();

        targetRadSpeed = (targetSpeed / MAX_EXPERIMENTAL_SPEED_IN_METERS * MAX_RAD_SPEED) * 1.01;

        collector.servos.setAngle(targetServoPos);

        if(!joystickActivityClass.buttonX) {
            double barabanPos;
            double pusherPos = PUSHER_START_POS;

            if(joystickActivityClass.tDpadRightPressed == 1){
                cellNum++;
                cellNum = Range.clip(cellNum, 0, 2);

                joystickActivityClass.tDpadRightPressed = 0;
            }
            if(joystickActivityClass.tDpadLeftPressed == 1){
                cellNum--;
                cellNum = Range.clip(cellNum, 0, 2);

                joystickActivityClass.tDpadLeftPressed = 0;
            }

            if(joystickActivityClass.tDpadUpPressed == 0){
                pusherPos = PUSHER_START_POS;
            } else if (joystickActivityClass.tDpadUpPressed == 1) {
                pusherPos = PUSHER_PREFIRE_POS;
            } else if(joystickActivityClass.tDpadUpPressed == 2) {
                pusherPos = PUSHERHOR_ENDING_POS;
            }else {
                joystickActivityClass.tDpadUpPressed = 0;
            }

            collector.servos.setPusherHor(pusherPos);

            if(isPushHorEnded() && collector.servos.curPusherHorPos <= PUSHER_PREFIRE_POS){
                rotateBaraban(cellNum);
            }

            if(isRotateEndedToLoad()){//Сначала ждём проворота барабана
                if (collector.colorSensorClass.colorState == ColorSensorClass.ColorSensorState.Artifact_Detected ){
                    collector.digitalCellsClass.setColor(collector.colorSensorClass.getArtifactColor());
                }else {
                    collector.digitalCellsClass.deleteColorFromCell();
                }
            }

            if (joystickActivityClass.bumperRight) {
                collector.motors.setSpeedFlyWheel(targetRadSpeed);
            }else {
                collector.motors.setSpeedFlyWheel(0);
            }

            if(joystickActivityClass.bumperLeft && collector.servos.curPusherHorPos == PUSHER_START_POS){
                collector.motors.onIntake();

            }else if(joystickActivityClass.tLeftBumperPressed != 0 && joystickActivityClass.tLeftBumperPressed % 2 == 0 ) {
                collector.motors.reverseInTake();

                if(collector.motors.getRunTimeIntake().seconds() > REVERSE_DELAY){
                    joystickActivityClass.tLeftBumperPressed = 0;
                }
            }else {
                collector.motors.offIntake();
            }

            if(collector.digitalCellsClass.getArtifactCount() == 3){
                loadState = LoadState.Idle;
            }else {
                loadState = LoadState.Prepare;
            }
            generalState = GeneralState.LoadLogic;

            fireState = FireState.Prepare;
        }else{
            joystickActivityClass.tDpadUpPressed = 0;
            joystickActivityClass.bumperRight = false;
            joystickActivityClass.bumperLeft = false;
            joystickActivityClass.tLeftBumperPressed = 0;
            joystickActivityClass.tRightBumperPressed = 0;

            if(joystickActivityClass.playersGamepad.back){
                collector.motors.reverseInTake();
                flag = 1;
                return;
            }else {
                if(flag == 1) collector.motors.offIntake();
                flag = 0;
               }

            switch (generalState){
                case LoadLogic:
                    collector.motors.setSpeedFlyWheel(targetRadSpeed / 2);

                    switch (loadState) {
                        case Prepare:
                            collector.motors.offIntake();
                            collector.servos.setPusherHor(PUSHER_START_POS);

                            if(isPushHorEnded()){
                                loadState = LoadState.Find;
                            }
                            break;

                        case Find:
                            int targCell = collector.digitalCellsClass.getEmptyCell();

                            rotateBaraban(targCell);

                            if (isRotateEndedToLoad()){
                                loadState = LoadState.Load;
                            }
                            break;

                        case Load:
                            if (collector.colorSensorClass.colorState == ColorSensorClass.ColorSensorState.Artifact_Detected){
                                collector.motors.offIntake();
                                collector.digitalCellsClass.setColor(collector.colorSensorClass.getArtifactColor());
                                loadState = LoadState.Idle;
                            }else collector.motors.onIntake();

                            break;

                        case Idle:
                            if(isLoadEnded())
                            {
                                collector.motors.reverseInTake();
                                if (collector.motors.getRunTimeFlyWheel().seconds() > REVERSE_DELAY) {
                                    collector.motors.offIntake();

                                    generalState = GeneralState.FireLogic;
                                    fireState = FireState.Prepare;
                                }
                            }
                            else
                            {
                                loadState = LoadState.Find;
                            }
                            break;

                        default:
                            break;
                    }

                    break;

                case FireLogic:
                    collector.motors.setSpeedFlyWheel(targetRadSpeed);

                    if (collector.motors.getRunTimeFlyWheel().seconds() < 0.15 || collector.motors.flyWheelStates == CollectorMotors.FlyWheelStates.Unready){
                        if(fireState == FireState.Fire && anotherStates == AnotherStates.Push) return;
                    }

                    switch (fireState) {
                        case Prepare:
                            collector.servos.setPusherHor(PUSHER_PREFIRE_POS);

                            fireState = FireState.Find;
                            break;

                        case Find:
//                            int targetCellIndex = 3 - collector.digitalCellsClass.getArtifactCount(); // 3→0, 2→1, 1→2
//                            int targetColor = collector.digitalCellsClass.getRandomizedArtifact()[targetCellIndex];
//
//                            int targCell = collector.digitalCellsClass.findNeededCell(targetColor);

                            int targCell = collector.digitalCellsClass.getFullCell();

                            rotateBaraban(targCell);

                            if (isRotateEndedToFire()){
                                fireState = FireState.Fire;
                            }
                            break;

                        case Fire:
                            switch (anotherStates){
                                case Push:
                                    collector.servos.setPusherVer(PUSHERVER_ENDING_POS);

                                    if (!(vyrState == PositionRobotController.VyrState.Far_from_it
                                            || moveState == OdometryClass.MoveState.High_speed
                                            || rotateState == OdometryClass.RotateState.High_speed))
                                    {
                                        if(isPushVerEnded()){
                                            collector.servos.setPusherHor(PUSHERHOR_ENDING_POS);

                                            if (isPushHorEnded()) {
                                                anotherStates = AnotherStates.Back;
                                            }
                                        }
                                    }
                                    break;

                                case Back:
                                    collector.servos.setPusherVer(PUSHERVER_START_POS);
                                    collector.servos.setPusherHor(PUSHER_PREFIRE_POS);

                                    if (isPushHorEnded()) {
                                        if (collector.colorSensorClass.colorState == ColorSensorClass.ColorSensorState.No_Artifact_Detected)
                                        {
                                            collector.digitalCellsClass.deleteColorFromCell();
                                            fireState = FireState.Idle;
                                        }

                                        anotherStates = AnotherStates.Push;
                                    }
                                    break;
                            }
                            break;

                        case Idle:
                            if (collector.digitalCellsClass.getArtifactCount() == 0)
                            {
                                generalState = GeneralState.LoadLogic;
                                loadState = LoadState.Prepare;
                            }
                            else fireState = FireState.Find;

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
    public boolean isRotateEndedToLoad(){
        return collector.servos.runTimeBaraban.seconds() > collector.servos.barabanDelay ;
    }
    public boolean isRotateEndedToFire(){
        return collector.servos.runTimeBaraban.seconds() > collector.servos.barabanDelay / 2.0;
    }
    public boolean isPushHorEnded(){
        return collector.servos.runTimePusherHor.seconds() > collector.servos.pusherHorDelay;
    }
    public boolean isArtifactDetected(){
        return collector.colorSensorClass.getTimeFromDetect().seconds() > DETECT_DELAY;
    }
    public boolean isPushVerEnded(){
        return collector.servos.runTimePusherVer.seconds() > collector.servos.pusherVerDelay;
    }
    public void rotateBaraban(int targCell){
        double targPos;

        switch (targCell){
            case 0:
                targPos = BARABAN_CELL0_POS;
                break;
            case 1:
                targPos = BARABAN_CELL1_POS;
                break;
            default:
                targPos = BARABAN_CELL2_POS;
                break;
        }

        collector.servos.setBaraban(targPos);
    }
    private void calcAngleToPos(){
        double rampAngle = Range.clip(90 - Math.toDegrees(targetAngle), MIN_ANGLE, MAX_ANGLE);

        targetServoPos =  (MAX_ANGLE - rampAngle) * (185 / 23) / 270;

        targetServoPos = Math.round(targetServoPos * Math.pow(10, 2)) / Math.pow(10, 2);
    }
    private void calcAngle(){
        double alpha = Math.toRadians(theta);

        targetAngle =  Math.atan(Math.tan(alpha) + 2 * (73) / range);

        targetAngle = Math.round(targetAngle * Math.pow(10, 1)) / Math.pow(10, 1);
    }
    private void calcSpeed(){
        double alpha = Math.toRadians(theta);

        targetSpeed =  Math.sqrt(Math.abs(981 * range / ((Math.tan(alpha) + Math.tan(targetAngle)) * Math.pow(Math.cos(targetAngle), 2)))) / 100;

        targetSpeed = Math.round(targetSpeed * Math.pow(10, 1)) / Math.pow(10, 1);
    }
    private boolean isLoadEnded(){
        return collector.digitalCellsClass.isMaxed;
    }
    @Override
    public void showData(){
        telemetry.addLine("===AUTO PLAYER===");
        telemetry.addData("Automatic state", generalState.toString());

        if(generalState == GeneralState.LoadLogic){
            telemetry.addData("Load state", loadState.toString());
        }else telemetry.addData("Fire state", fireState.toString());

        telemetry.addData("Another state", anotherStates.toString());
        if(fireState == FireState.Fire && anotherStates == AnotherStates.Push)
        {
            telemetry.addData("FlyWheel", collector.motors.flyWheelStates.toString());
            telemetry.addData("Randomize", randomizeStatus.toString());
            telemetry.addData("VyrState", vyrState.toString());
            telemetry.addData("MoveState", moveState.toString());
            telemetry.addData("RotateState", rotateState.toString());
        }

        telemetry.addLine();
    }

}
