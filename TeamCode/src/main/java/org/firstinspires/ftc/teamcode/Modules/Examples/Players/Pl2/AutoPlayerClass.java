package org.firstinspires.ftc.teamcode.Modules.Examples.Players.Pl2;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Modules.Joysticks.JoystickActivityClass;
import org.firstinspires.ftc.teamcode.Modules.Examples.Players.PlayerClass;
import org.firstinspires.ftc.teamcode.Robot.RobotClass;
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

        generalState = GeneralState.Load;
        loadState = LoadState.Prepare_to_load;
        fireState = FireState.Prepare_to_fire;
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
    public enum GeneralState {
        Load,
        Fire
    }
    public enum LoadState{
        Idle,
        Load_and_check,
        Reverse,
        Prepare_to_load,
        Find_empty_cell,
        Move_to_0_cell,
        Move_to_1_cell,
        Move_to_2_cell,
        On_cell
    }
    public enum FireState{
        Push_artifact,
        Pusher_back,
        Find_needed_cell,
        Idle,
        Prepare_to_fire,
        Move_to_0_cell,
        Move_to_1_cell,
        Move_to_2_cell,
        On_cell_check_states,
        Check_emptiness
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

        targetRadSpeed = (targetSpeed / MAX_EXPERIMENTAL_SPEED_IN_METERS * MAX_RAD_SPEED) * 1.03;

//        if (range > 200) theta = 75;
//        else theta = 30;
//        if(innerTime.seconds() > END_TIME + 5){
//            return;
//        } else if (innerTime.seconds() > END_TIME) {
//            generalState = GeneralState.Fire;
//        }else if(innerTime.seconds() > END_TIME - 1){
//            joystickActivityClass.buttonX = true;
//        }

        if(!joystickActivityClass.buttonX) {
            double barabanPos;
            double pusherPos = PUSHER_START_POS;

            collector.servos.setAngle(targetServoPos);

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

            switch (cellNum){
                case 0:
                    barabanPos = BARABAN_CELL0_POS;
                    break;
                case 1:
                    barabanPos = BARABAN_CELL1_POS;
                    break;
                default:
                    barabanPos = BARABAN_CELL2_POS;
                    break;
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

//            if(joystickActivityClass.tDpadRightPressed == 0){
//                pusherPos = PUSHER_START_POS;
//            } else if (joystickActivityClass.tDpadRightPressed == 1) {
//                pusherPos = PUSHER_PREFIRE_POS;
//            } else if(joystickActivityClass.tDpadRightPressed == 2) {
//                pusherPos = PUSHERHOR_ENDING_POS;
//            }else {
//                joystickActivityClass.tDpadRightPressed = 0;
//            }

//            if(isPushEnded()){
//                if(pusherPos == PUSHER_ENDING_POS) {
//                    joystickActivityClass.tDpadUpPressed = 1;
//                    pusherPos = PUSHER_PREFIRE_POS;
//                }
//            }
            collector.servos.setPusherHor(pusherPos);
//
            if(isPushHorEnded() && collector.servos.curPusherHorPos <= PUSHER_PREFIRE_POS){
                collector.servos.setBaraban(barabanPos);
            }

            if(isRotateEndedToLoad()){//Сначала ждём проворота барабана
                if (collector.colorSensorClass.colorState == ColorSensorClass.ColorSensorState.Artifact_Detected ){
                    collector.digitalCellsClass.setColor(collector.colorSensorClass.getArtifactColor());
                }else {
                    collector.digitalCellsClass.deleteColorFromCell();
                }
            }
//
            if (joystickActivityClass.bumperRight) {
                collector.motors.setSpeedFlyWheel(targetRadSpeed);
            }else {
                collector.motors.setSpeedFlyWheel(0);
            }
//            if (joystickActivityClass.dpad_Left) {
//                collector.motors.setSpeed(targetRadSpeed);
//            }else {
//                collector.motors.setSpeed(0);
//            }
//
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
                loadState = LoadState.Prepare_to_load;
            }
            generalState = GeneralState.Load;

            fireState = FireState.Prepare_to_fire;
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
                case Load:
                    switch (loadState) {
                        case Prepare_to_load:
                            collector.motors.offIntake();
                            collector.servos.setPusherHor(PUSHER_START_POS);

                            if(isPushHorEnded()){
                                loadState = LoadState.Find_empty_cell;
                            }
                            break;

                        case Find_empty_cell:
                            int cell = collector.digitalCellsClass.getNextBarabanPos();

                            telemetry.addData("cell", cell);
                            switch (cell){
                                case 0:
                                    loadState = LoadState.Move_to_0_cell;
                                    break;
                                case 1:
                                    loadState = LoadState.Move_to_1_cell;
                                    break;
                                case 2:
                                    loadState = LoadState.Move_to_2_cell;
                                    break;
                            }
//                            if(joystickActivityClass.dpad_Up){
//
//                                joystickActivityClass.dpad_Up = false;
//                            }

                            break;

                        case Move_to_0_cell:
                            collector.servos.setBaraban(BARABAN_CELL0_POS);
                            cellNum = 0;


                            if(isRotateEndedToLoad()){
                                telemetry.addData("isRotateEndedToLoad()", isRotateEndedToLoad());
                                telemetry.addData("cellNum", cellNum);
                                loadState = LoadState.On_cell;
//                                if(joystickActivityClass.dpad_Up){
//
//                                    joystickActivityClass.dpad_Up = false;
//                                }
                            }
                            break;

                        case Move_to_1_cell:
                            collector.servos.setBaraban(BARABAN_CELL1_POS);
                            cellNum = 1;

                            if(isRotateEndedToLoad()){
                                telemetry.addData("isRotateEndedToLoad()", isRotateEndedToLoad());
                                telemetry.addData("cellNum", cellNum);
                                loadState = LoadState.On_cell;
//                                if(joystickActivityClass.dpad_Up){
//
//                                    joystickActivityClass.dpad_Up = false;
//                                }
                            }
                            break;

                        case Move_to_2_cell:
                            collector.servos.setBaraban(BARABAN_CELL2_POS);
                            cellNum = 2;

                            if(isRotateEndedToLoad()){
                                telemetry.addData("isRotateEndedToLoad()", isRotateEndedToLoad());
                                telemetry.addData("cellNum", cellNum);
                                loadState = LoadState.On_cell;
//                                if(joystickActivityClass.dpad_Up){
//
//                                    joystickActivityClass.dpad_Up = false;
//                                }
                            }
                            break;

                        case On_cell:
                            collector.motors.onIntake();

                            if(collector.colorSensorClass.colorState == ColorSensorClass.ColorSensorState.Artifact_Detected){
                                telemetry.addData("colorState", collector.colorSensorClass.colorState);
                                collector.motors.offIntake();
                                loadState = LoadState.Load_and_check;
//                                if(joystickActivityClass.dpad_Up){
//
//                                    joystickActivityClass.dpad_Up = false;
//                                }
                            }
                            break;

                        case Load_and_check:
                            telemetry.addData("condition", collector.colorSensorClass.getTimeFromDetect().seconds() > DETECT_DELAY);
                            telemetry.addData("setColor", collector.colorSensorClass.getArtifactColor());
                            telemetry.addData("attempts", attempts);

                            if (collector.colorSensorClass.getTimeFromDetect().seconds() > DETECT_DELAY){
                                collector.digitalCellsClass.setColor(collector.colorSensorClass.getArtifactColor());
                                if(collector.digitalCellsClass.getArtifactCount() == 3){
                                    loadState = LoadState.Idle;
                                }else {
                                    loadState = LoadState.Find_empty_cell;
                                }
//                                if(joystickActivityClass.dpad_Up){
//
//
//                                    joystickActivityClass.dpad_Up = false;
//                                }
                            }else attempts++;

                            if(attempts == 3) {
                                loadState = LoadState.On_cell;
                                attempts = 0;
                            }
                            break;

//                        case Reverse:
//                            collector.motors.reverseInTake();
//
//                            if(collector.motors.runTimeIntake.seconds() > delayToReverse){
//                                collector.motors.offIntake();
//                                loadState = LoadState.Find_empty_cell;
//                            }
//                            break;

                        case Idle:
                            collector.motors.reverseInTake();
                            if (collector.motors.getRunTimeFlyWheel().seconds() > REVERSE_DELAY) {
                                collector.motors.offIntake();

                                collector.motors.preFireSpeedFlyWheel();
                                collector.servos.setPusherHor(PUSHER_PREFIRE_POS);

                                generalState = GeneralState.Fire;
                                fireState = FireState.Prepare_to_fire;
                                innerTime.reset();
                            }

                            break;

                        default:
                            break;
                    }

                    break;

                case Fire:
                    collector.servos.setAngle(targetServoPos);
                    collector.motors.setSpeedFlyWheel(targetRadSpeed);
//
                    if (collector.motors.getRunTimeFlyWheel().seconds() < 0.15 || collector.motors.flyWheelStates == CollectorMotors.FlyWheelStates.Unready){
                        if(fireState == FireState.On_cell_check_states) return;
                    }

                    switch (fireState) {
                        case Prepare_to_fire:
                            collector.servos.setPusherHor(PUSHER_PREFIRE_POS);

                            if (isPushHorEnded()) {
                                fireState = FireState.Find_needed_cell;
                            }
                            break;

                        case Find_needed_cell:
                            if (collector.digitalCellsClass.getArtifactCount() == 0) {
                                fireState = FireState.Idle;
                                break;
                            }

                            // Определяем целевую ячейку (0, 1 или 2)
                            int targetCellIndex = 3 - collector.digitalCellsClass.getArtifactCount(); // 3→0, 2→1, 1→2
                            int targetColor = collector.digitalCellsClass.getRandomizedArtifact()[targetCellIndex];

                            int cell = collector.digitalCellsClass.findNeededCell(targetColor);

                            switch (cell){
                                case 0:
                                    fireState = FireState.Move_to_0_cell;
                                    break;
                                case 1:
                                    fireState = FireState.Move_to_1_cell;
                                    break;
                                case 2:
                                    fireState = FireState.Move_to_2_cell;
                                    break;
                            }
                            break;

                        case Move_to_0_cell:
                            collector.servos.setBaraban(BARABAN_CELL0_POS);
                            cellNum = 0;

                            if(isRotateEndedToFire()){
                                fireState = FireState.On_cell_check_states;
                            }
                            break;

                        case Move_to_1_cell:
                            collector.servos.setBaraban(BARABAN_CELL1_POS);
                            cellNum = 1;

                            if(isRotateEndedToFire()){
                                fireState = FireState.On_cell_check_states;
                            }
                            break;

                        case Move_to_2_cell:
                            collector.servos.setBaraban(BARABAN_CELL2_POS);
                            cellNum = 2;

                            if(isRotateEndedToFire()){
                                fireState = FireState.On_cell_check_states;
                            }
                            break;

                        case On_cell_check_states:
                            if(vyrState == PositionRobotController.VyrState.Far_from_it || vyrState == PositionRobotController.VyrState.Near_from_it
                                    || moveState == OdometryClass.MoveState.High_speed
                                    || rotateState == OdometryClass.RotateState.High_speed) {
                                return;//Программе дальше нет смысла идти пока не выполнятся условия
                            }else fireState = FireState.Push_artifact;

                            break;

                        case Push_artifact:
                            collector.servos.setPusherHor(PUSHERHOR_ENDING_POS);

                            if (isPushHorEnded()) {
                                fireState = FireState.Pusher_back;
                            }
                            break;

                        case Pusher_back:
                            collector.servos.setPusherHor(PUSHER_PREFIRE_POS);

                            if (isPushHorEnded()) {
                                fireState = FireState.Check_emptiness;
                            }
                            break;

                        case Check_emptiness:
                            if (collector.colorSensorClass.colorState == ColorSensorClass.ColorSensorState.No_Artifact_Detected) {
                                collector.digitalCellsClass.deleteColorFromCell();
                                fireState = FireState.Find_needed_cell;
                            } else {
                                fireState = FireState.On_cell_check_states;
                            }

                            break;

                        case Idle:
                            collector.motors.offFLyWheel();
                            collector.servos.setPusherHor(PUSHER_START_POS);

                            generalState = GeneralState.Load;
                            loadState = LoadState.Prepare_to_load;
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
        return collector.servos.runTimePusherVer.seconds() > PUSHERVER_DELAY;
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
    @Override
    public void showData(){
        telemetry.addLine("===AUTO PLAYER===");
        telemetry.addData("Automatic state", generalState.toString());

        if(generalState == GeneralState.Load){
            telemetry.addData("Load state", loadState.toString());
        }else telemetry.addData("Fire state", fireState.toString());

        telemetry.addData("FlyWheel", collector.motors.flyWheelStates.toString());
        telemetry.addData("Randomize", randomizeStatus.toString());
        telemetry.addData("VyrState", vyrState.toString());
        telemetry.addData("MoveState", moveState.toString());
        telemetry.addData("RotateState", rotateState.toString());
        telemetry.addLine();
    }

}
