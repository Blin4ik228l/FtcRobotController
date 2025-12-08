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
        Check_color,
        Reverse,
        Rotate_baraban,

        Baraban_moving_to_0,
        Baraban_at_0,
        Baraban_moving_to_1,
        Baraban_at_1,
        Baraban_moving_to_2,
        Baraban_at_2,
        Prepare_to_load
    }
    public enum FireState{
        Baraban_moving_to_0,
        Baraban_moving_to_1,
        Baraban_moving_to_2,
        Baraban_at_pos,
        Pusher_back,
        Find_and_turn,
        Idle,
        Pusher_start,
        Prepare_to_fire,
        Set_speed
    }

    @Override
    public void execute(){
        double delayToBaraban = BARABAN_DELAY;
        double delayToPusher = PUSHER_DELAY;
        double delayToReverse = REVERSE_DELAY;

        curAngle = getAngle(range);
        targetSpeed = getSpeed(range, curAngle);
        curVelRad = targetSpeed / MAX_EXPERIMENTAL_SPEED_IN_METERS * MAX_RAD_SPEED;

        collector.servos.setAngle(findNeededPosAngle(curAngle));

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
                loadState = LoadState.Prepare_to_load;

            }else if(joystickActivity.tLeftBumperPressed != 0 && joystickActivity.tLeftBumperPressed % 2 == 0 ) {
                collector.motors.reverseInTake();

                if(collector.motors.runTimeIntake.seconds() > delayToReverse){
                    joystickActivity.tLeftBumperPressed = 0;
                }
                loadState = LoadState.Prepare_to_load;
            }else {
                collector.motors.offIntake();
                loadState = LoadState.Prepare_to_load;
            }


            if(artifactCount == 3){
                automaticState = CollectorState.Fire;
            }else {
                automaticState = CollectorState.Load;
            }

            fireState = FireState.Prepare_to_fire;
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
                        case Prepare_to_load:
                            collector.motors.offIntake();
                            collector.servos.setPusher(PUSHER_START_POS);

                            if(collector.servos.runTimePusher.seconds() > delayToPusher){
                                loadState = LoadState.Check_color;
                            }
                            break;
                        case Check_color:
                            collector.motors.onIntake();

                            if (collector.colorSensor.colorState == ColorSensor.ColorSensorState.Artifact_Detected) {
                                collector.motors.offIntake();

                                digitalCells.setColor(collector.colorSensor.artifactColor);
                                digitalCells.checkNumberOfArtifacts();

                                if(digitalCells.artifactCount == 3){
                                    loadState = LoadState.Idle;

                                }else {
                                    loadState = LoadState.Reverse;
                                }

                            }
                            break;
                        case Reverse:
                            collector.motors.reverseInTake();

                            if(collector.motors.runTimeIntake.seconds() > 0.15){
                                collector.motors.offIntake();
                                loadState = LoadState.Rotate_baraban;
                            }
                            break;
                        case Rotate_baraban:
                            double barabanPos = digitalCells.getBarabanPos();

                            collector.servos.setBaraban(barabanPos);
                            if(isRotateEnded(delayToBaraban)){
                                collector.motors.onIntake();
                                loadState = LoadState.Check_color;
                            }
                            break;

                        case Idle:
                            collector.motors.reverseInTake();
                            if (collector.motors.runTimeIntake.seconds() > delayToReverse) {
                                collector.motors.offIntake();

                                automaticState = CollectorState.Fire;
                                fireState = FireState.Prepare_to_fire;
                            }
                            break;

                        default:
                            break;
                    }

                    break;

                case Fire:
                    switch (fireState) {
                        case Prepare_to_fire:
                            collector.motors.preFireSpeedFlyWheel();

                            collector.servos.setPusher(PUSHER_PREFIRE_POS);

                            if (collector.servos.runTimePusher.seconds() > delayToPusher) {
                                fireState = FireState.Find_and_turn;
                            }
                            break;

                        case Find_and_turn:
                            if (artifactCount == 0) {
                                fireState = FireState.Idle;
                                break;
                            }

                            // Определяем целевую ячейку (0, 1 или 2)
                            int targetCellIndex = 3 - artifactCount; // 3→0, 2→1, 1→2
                            int targetColor = randomizedArtifacts[targetCellIndex];

                            double targetPos = digitalCells.findNeededArtifactPos(targetColor);

                            collector.servos.setBaraban(targetPos);
                            if(isRotateEnded(delayToBaraban)){
                                fireState = FireState.Set_speed;
                            }
                            break;


                        case Set_speed:
                            double DELTA = 3e-3;
                            collector.motors.setSpeed(curVelRad);

                            if(Math.abs(collector.motors.curOverallVel - curVelRad) < DELTA){
                                fireState = FireState.Baraban_at_pos;
                            }
                            break;

                        case Baraban_at_pos:
                            if(!isRandomizeWasDetected() || !isAllowFire() || !isRobotHaveMinVel()){
                                fireState = FireState.Set_speed;
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
                                    fireState = FireState.Find_and_turn;
                                } else {
                                    fireState = FireState.Baraban_at_pos;
                                }
                            }
                            break;

                        case Idle:
                            collector.motors.offFLyWheel();

                            automaticState = CollectorState.Load;
                            loadState = LoadState.Prepare_to_load;
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
        return isVyrCompleted;
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
