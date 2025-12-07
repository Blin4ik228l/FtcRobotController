package org.firstinspires.ftc.teamcode.Modules.Examples.Players.Pl2;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Modules.Examples.Players.JoystickActivity;
import org.firstinspires.ftc.teamcode.Modules.Examples.Players.PlayerClass;
import org.firstinspires.ftc.teamcode.Robot.RobotClass;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.CollectorParts.ColorSensor;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.CollectorParts.DigitalCells;

public class AutomaticClass extends PlayerClass{
    public AutomaticClass(JoystickActivity joystickActivity, RobotClass.Collector collector, OpMode op) {
        super(joystickActivity, op.telemetry);
        this.collector = collector;

        digitalCells = new DigitalCells(collector.servos, op);
    }
    public RobotClass.Collector collector;
    private final DigitalCells digitalCells;
    public int[] randomizedArtifacts = new int[3];
    public boolean isVyrCompleted;
    public double range;
    public double headVel;
    public int artifactCount;
    public double vel;
    public void setFields(
            int[] randomizedArtifacts, boolean isVyrCompleted,
                          double range, double headVel, double vel){

        this.randomizedArtifacts = randomizedArtifacts;
        this.isVyrCompleted = isVyrCompleted;
        this.range = range;
        this.headVel = headVel;
        this.vel = vel;
    }
    double curAngle;
    double targetSpeed, curVelRad;
    int count;
    boolean isLoadEnded;

    public CollectorState automaticState;
    public LoadState loadState;
    public FireState fireState;
    public enum CollectorState{
        Load,
        Fire
    }
    public enum LoadState{
        Idle,
        Idle2,
        Baraban_moving_to_0,
        Baraban_at_0,
        Baraban_moving_to_1,
        Baraban_at_1,
        Baraban_moving_to_2,
        Baraban_at_2,
        Pusher_start
    }
    public enum FireState{
        Baraban_moving_to_0,
        Baraban_moving_to_1,
        Baraban_moving_to_2,
        Baraban_at_pos,
        Pusher_back,
        Find_Color,
        Idle,
        Pusher_start,
        Pusher_prefire,
        Set_Speed
    }

    @Override
    public void execute(){
        double delayToBaraban = 0.3;
        double delayToPusher = 0.6;
        double delayToReverse = 1;

        curAngle = getAngle(range);
        targetSpeed = getSpeed(range, curAngle);
        curVelRad = targetSpeed / MAX_EXPERIMENTAL_SPEED_IN_METERS * MAX_RAD_SPEED;

        collector.servos.setAngle(findNeededPosAngle(curAngle));

        digitalCells.checkNumberOfArtifacts();

        if(!joystickActivity.buttonX) {
            double barabanPos = BARABAN_CELL0_POS;
            double pusherPos = PUSHER_START_POS;

            if(joystickActivity.tDpadRightPressed == 1){
                count ++;
                count = Range.clip(count, 0, 3);

                joystickActivity.tDpadRightPressed = 0;
            }
            if(joystickActivity.tDpadLeftPressed == 1){
                count --;
                count = Range.clip(count, 0, 3);

                joystickActivity.tDpadLeftPressed = 0;
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

            if(joystickActivity.tDpadUpPressed == 0){
                pusherPos = PUSHER_START_POS;
            } else if (joystickActivity.tDpadUpPressed == 1) {
                pusherPos = PUSHER_PREFIRE_POS;
            } else if(joystickActivity.tDpadUpPressed == 2) {
                pusherPos = PUSHER_ENDING_POS;
            }else {
                joystickActivity.tDpadUpPressed = 0;
            }

            if(artifactCount == 3){
                isLoadEnded = true;
            }else if (artifactCount == 0) {
                isLoadEnded = false;
            }

            if(isLoadEnded){
                // Определяем целевую ячейку (0, 1 или 2)
                int targetCellIndex = 3 - artifactCount; // 3→0, 2→1, 1→2
                int targetColor = randomizedArtifacts[targetCellIndex];

                barabanPos = digitalCells.findNeededArtifactPos(targetColor);
            }

            collector.servos.setPusher(pusherPos);

            if(collector.servos.runTimePusher.seconds() > delayToPusher && collector.servos.curPusherPos <= PUSHER_PREFIRE_POS){
                collector.servos.setBaraban(barabanPos);

                if(isRotateEnded(delayToBaraban)){//Сначала ждём проворота барабана
                    if(!isLoadEnded){//Затем проверяем режим
                        if (collector.colorSensor.colorState == ColorSensor.ColorSensorState.Artifact_Detected ){
                            digitalCells.setColor(collector.colorSensor.artifactColor);
                        }
                    }else{
                        if (collector.colorSensor.colorState == ColorSensor.ColorSensorState.No_Artifact_Detected){
                            digitalCells.deleteColorFromCell();
                        }
                    }
                }
            }

            if (joystickActivity.bumperRight) {
                collector.motors.setSpeed(curVelRad);
            }else {
                collector.motors.setSpeed(0);
            }

            if(joystickActivity.bumperLeft && collector.servos.curPusherPos == PUSHER_START_POS){
                collector.motors.onIntake();
                loadState = LoadState.Idle2;

            }else if(joystickActivity.tLeftBumperPressed != 0 && joystickActivity.tLeftBumperPressed % 2 == 0 && collector.servos.curPusherPos == PUSHER_START_POS) {
                collector.motors.reverseInTake();

                if(collector.motors.runTimeIntake.seconds() > delayToReverse){
                    joystickActivity.tLeftBumperPressed = 0;
                }
                loadState = LoadState.Pusher_start;
            }else {
                collector.motors.offIntake();
                loadState = LoadState.Pusher_start;
            }

            automaticState = CollectorState.Load;

            fireState = FireState.Pusher_prefire;
        }else{
            joystickActivity.tDpadUpPressed = 0;
            joystickActivity.tDpadDownPressed = 0;
            joystickActivity.bumperRight = false;
            joystickActivity.bumperLeft = false;
            joystickActivity.tLeftBumperPressed = 0;
            joystickActivity.tRightBumperPressed = 0;
            count = 0;

            switch (automaticState){
                case Load:
                    switch (loadState) {
                        case Idle2:
                            collector.motors.reverseInTake();

                            if(collector.motors.runTimeIntake.seconds() > delayToReverse){
                                collector.motors.offIntake();
                                loadState = LoadState.Pusher_start;
                            }
                            break;

                        case Pusher_start:
                            collector.servos.setPusher(PUSHER_START_POS);

                            if(collector.servos.runTimePusher.seconds() > delayToPusher){
                                if(artifactCount == 3){
                                    automaticState = CollectorState.Fire;
                                }else {
                                    collector.motors.onIntake();
                                    loadState = LoadState.Baraban_moving_to_0;
                                }
                            }
                            break;

                        case Baraban_moving_to_0:
                            collector.servos.setBaraban(BARABAN_CELL0_POS);

                            if (isRotateEnded(delayToBaraban)) {
                                loadState = LoadState.Baraban_at_0;
                            }
                            break;

                        case Baraban_at_0:
                            if (collector.colorSensor.colorState == ColorSensor.ColorSensorState.Artifact_Detected) {
                                digitalCells.setColor(collector.colorSensor.artifactColor);
                                loadState = LoadState.Baraban_moving_to_1;
                            }
                            break;

                        case Baraban_moving_to_1:
                            collector.servos.setBaraban(BARABAN_CELL1_POS);

                            if (isRotateEnded(delayToBaraban)) {
                                loadState = LoadState.Baraban_at_1;
                            }
                            break;

                        case Baraban_at_1:
                            if (collector.colorSensor.colorState == ColorSensor.ColorSensorState.Artifact_Detected) {
                                digitalCells.setColor(collector.colorSensor.artifactColor);
                                loadState = LoadState.Baraban_moving_to_2;
                            }
                            break;

                        case Baraban_moving_to_2:
                            collector.servos.setBaraban(BARABAN_CELL2_POS);

                            if (isRotateEnded(delayToBaraban)) {
                                loadState = LoadState.Baraban_at_2;
                            }
                            break;

                        case Baraban_at_2:
                            if (collector.colorSensor.colorState == ColorSensor.ColorSensorState.Artifact_Detected) {
                                digitalCells.setColor(collector.colorSensor.artifactColor);
                                loadState = LoadState.Idle;
                            }
                            break;

                        case Idle:
                            collector.motors.reverseInTake();

                            if (collector.motors.runTimeIntake.seconds() > delayToReverse) {
                                collector.motors.offIntake();
                                automaticState = CollectorState.Fire;
                            }
                            break;

                        default:
                            break;
                    }

                    break;

                case Fire:
                    switch (fireState) {
                        case Pusher_prefire:
                            collector.servos.setPusher(PUSHER_PREFIRE_POS);

                            if (collector.servos.runTimePusher.seconds() > delayToPusher) {
                                collector.motors.preFireSpeedFlyWheel();

                                fireState = FireState.Find_Color;
                            }
                            break;

                        case Find_Color:
                            if (artifactCount == 0) {
                                fireState = FireState.Pusher_start;
                                break;
                            }

                            // Определяем целевую ячейку (0, 1 или 2)
                            int targetCellIndex = 3 - artifactCount; // 3→0, 2→1, 1→2
                            int targetColor = randomizedArtifacts[targetCellIndex];

                            double targetPos = digitalCells.findNeededArtifactPos(targetColor);

                            // Выбираем состояние движения
                            if (targetPos == BARABAN_CELL2_POS) {
                                fireState = FireState.Baraban_moving_to_2;
                            } else if (targetPos == BARABAN_CELL1_POS) {
                                fireState = FireState.Baraban_moving_to_1;
                            } else {
                                fireState = FireState.Baraban_moving_to_0;
                            }
                            break;

                        case Baraban_moving_to_0:
                            collector.servos.setBaraban(BARABAN_CELL0_POS);

                            if (isRotateEnded(delayToBaraban)) {
                                fireState = FireState.Set_Speed;
                            }
                            break;

                        case Baraban_moving_to_1:
                            collector.servos.setBaraban(BARABAN_CELL1_POS);

                            if (isRotateEnded(delayToBaraban)) {
                                fireState = FireState.Set_Speed;
                            }
                            break;

                            case Baraban_moving_to_2:
                            collector.servos.setBaraban(BARABAN_CELL2_POS);

                            if (isRotateEnded(delayToBaraban)) {
                                fireState = FireState.Set_Speed;
                            }
                            break;

                        case Set_Speed:
                            double DELTA = 3e-3;
                            collector.motors.setSpeed(curVelRad);

                            if(Math.abs(collector.motors.curOverallVel - curVelRad) < DELTA){
                                fireState = FireState.Baraban_at_pos;
                            }
                            break;

                        case Baraban_at_pos:
                            if(!isRandomizeWasDetected() || !isAllowFire() || !isRobotHaveMinVel()){
                                fireState = FireState.Set_Speed;
                                break;}

                            collector.servos.setPusher(1);

                            if (collector.servos.runTimePusher.seconds() > delayToPusher) {
                                fireState = FireState.Pusher_back;
                            }
                            break;

                        case Pusher_back:
                            collector.servos.setPusher(PUSHER_PREFIRE_POS);

                            if (collector.servos.runTimePusher.seconds() > delayToPusher) {
                                if (collector.colorSensor.colorState == ColorSensor.ColorSensorState.No_Artifact_Detected) {
                                    digitalCells.deleteColorFromCell();
                                    fireState = FireState.Find_Color;
                                } else {
                                    fireState = FireState.Baraban_at_pos;
                                }
                            }
                            break;

                        case Pusher_start:
                            collector.servos.setPusher(PUSHER_START_POS);

                            if (collector.servos.runTimePusher.seconds() > delayToPusher) {
                                fireState = FireState.Idle;
                            }
                            break;

                        case Idle:
                            collector.motors.offFLyWheel();

                            automaticState = CollectorState.Load;
                            loadState = LoadState.Pusher_start;
                            fireState = FireState.Pusher_prefire;

                            joystickActivity.buttonBack = false;
                            break;

                        default:
                            break;
                    }

                    break;
                default:
                    throw new IllegalStateException("Unexpected value: " + automaticState);
            }
        }
    }

    public boolean isRotateEnded(double delayToBaraban){
        return collector.servos.runTimeBaraban.seconds() > delayToBaraban || collector.buttonClass.getState();
    }
    public double findNeededPosAngle(double curAngle){
        return ((90 - MAX_ANGLE) - (90 - Math.toDegrees(curAngle))) * (185 / 23) / 270;
    }
    double getAngle(double range){
        return Math.atan(Math.tan(Math.toRadians(60)) + 2 * (100 - 30) / range);
    }
    double getSpeed(double range, double angle){
        return Math.sqrt(981 * range / ((Math.tan(Math.toRadians(60)) + Math.tan(angle)) * Math.pow(Math.cos(angle), 2))) / 100;
    }
    public boolean isRobotHaveMinVel(){
        return Math.abs(headVel) < Math.toRadians(30);
    }
    public boolean isAllowFire(){
        return true;
//        return isVyrCompleted && joystickActivity.buttonY;
    }

    public boolean isRandomizeWasDetected(){
        return randomizedArtifacts[0] != 0;
    }
    @Override
    public void showData(){
        telemetry.addLine("=== AUTOMATIC ===");
        telemetry.addData("Target speed", "%.1fm/s", targetSpeed);
        telemetry.addData("Automatic state", automaticState.toString());
        telemetry.addData("Load state", loadState.toString());
        telemetry.addData("Fire state", fireState.toString());
        telemetry.addLine();
    }
    public String getColorFromNumber(double number){
        return number == 2 ? "Purple" : number == 1 ? "Green" : "Empty";
    }
}
