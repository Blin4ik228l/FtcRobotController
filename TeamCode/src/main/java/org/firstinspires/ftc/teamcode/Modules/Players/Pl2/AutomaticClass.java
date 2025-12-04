package org.firstinspires.ftc.teamcode.Modules.Players.Pl2;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Modules.Players.JoystickActivity;
import org.firstinspires.ftc.teamcode.Modules.Players.PlayerClass;
import org.firstinspires.ftc.teamcode.Robot.RobotClass;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.CollectorParts.ButtonClass;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.CollectorParts.ColorSensor;

public class AutomaticClass extends PlayerClass{
    public AutomaticClass(JoystickActivity joystickActivity, RobotClass.Collector collector, OpMode op) {
        super(joystickActivity, op.telemetry);
        this.collector = collector;

        cells = new Cells();
        timeFromLoad = new ElapsedTime();

        automaticState = CollectorState.Load;
        loadState = LoadState.Baraban_moving_to_0;
        fireState = FireState.Pusher_prefire;
    }
    public RobotClass.Collector collector;
    private final Cells cells;
    private final ElapsedTime timeFromLoad;
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
    double curVel;
    double curLength;
    double curHeight;

    public CollectorState automaticState;
    public LoadState loadState;
    public FireState fireState;
    public enum CollectorState{
        Load,
        Fire
    }
    public enum LoadState{
        Idle,
        Baraban_moving_to_0,
        Baraban_at_0,
        Baraban_moving_to_05,
        Baraban_at_05,
        Baraban_moving_to_1,
        Baraban_at_1,
        Pusher_start

    }
    public enum FireState{
        Baraban_moving_to_0,
        Baraban_moving_to_05,
        Baraban_moving_to_1,
        Baraban_at_pos,
        Pusher_back,
        Find_Color,
        Idle,
        Pusher_start,
        Pusher_prefire,
        Move_angle
    }

    @Override
    public void execute(){
        double delayToRotate = 0.2;
        double delayToBaraban = 0.3;
        double delayToPusher = 0.8;
        double delayToReverse = 1;
        double delayToAngle = 0.5;

        curAngle = getAngle3(range);
        curVel = getSpeed(range, curAngle);

        collector.servos.setAngle(findNeededPosAngle(curAngle));

        if(!joystickActivity.buttonX) {
            double barabanPos;
            double pusherPos = 0.0;

            if(joystickActivity.tDpadUpPressed == 0){
                if(collector.servos.runTimePusher.seconds() > 0.3 && collector.servos.curPusherPos <= 0.55){
                    collector.servos.setBaraban(0.0);

                    if(collector.colorSensor.colorState == ColorSensor.ColorSensorState.Artifact_Detected
                            && collector.servos.runTimeBaraban.seconds() > 0.5){
                        loadArtifactInCell(findNeededCell().numCell);

                    }
                }
            }else if(joystickActivity.tDpadUpPressed == 1){
                if(collector.servos.runTimePusher.seconds() > 0.3 && collector.servos.curPusherPos <= 0.55){
                    collector.servos.setBaraban(0.25);

                    if(collector.colorSensor.colorState == ColorSensor.ColorSensorState.Artifact_Detected
                            && collector.servos.runTimeBaraban.seconds() > 0.5){
                        loadArtifactInCell(findNeededCell().numCell);
                    }
                }

            }else if(joystickActivity.tDpadUpPressed == 2){
                if(collector.servos.runTimePusher.seconds() > 0.3 && collector.servos.curPusherPos <= 0.55){
                    collector.servos.setBaraban(0.5);

                    if(collector.colorSensor.colorState == ColorSensor.ColorSensorState.Artifact_Detected
                            && collector.servos.runTimeBaraban.seconds() > 0.5){
                        loadArtifactInCell(findNeededCell().numCell);

                    }
                }
            }else {
                joystickActivity.tDpadUpPressed = 0;
            }

            if(joystickActivity.tDpadDownPressed == 0){
                pusherPos = 0.0;
            } else if (joystickActivity.tDpadDownPressed == 1) {
                pusherPos = 0.50;
            } else if(joystickActivity.tDpadDownPressed == 2) {
                pusherPos = 1;
            }else {
                joystickActivity.tDpadDownPressed = 0;
            }

            collector.servos.setPusher(pusherPos);


            if (joystickActivity.bumperLeft) {
                collector.motors.setSpeed(curVel / 6.28 * 5);
            }else {
                collector.motors.setSpeed(0);
            }

            if(joystickActivity.bumperRight){
                collector.motors.onIntake();
            }else {
                collector.motors.offIntake();
            }

            automaticState = CollectorState.Load;
            loadState = LoadState.Pusher_start;
        }else{
            joystickActivity.tDpadUpPressed = 0;
            joystickActivity.tDpadDownPressed = 0;
            joystickActivity.bumperRight = false;
            joystickActivity.bumperLeft = false;

            checkNumberOfArtifacts();

            switch (automaticState){
                case Load:

                    switch (loadState) {
                        case Pusher_start:
                            collector.motors.offIntake();
                            collector.servos.setPusher(0.0);

                            if(collector.servos.runTimePusher.seconds() > 0.5){
                                loadState = LoadState.Baraban_at_0;
                            }
                            if(artifactCount == 3){
                                loadState = LoadState.Idle;
                            }

                            break;

                        case Baraban_moving_to_0:

                            collector.servos.setBaraban(0.0);

                            if (isRotateEnded(delayToBaraban)) {
                                loadState = LoadState.Baraban_at_0;
                            }
                            break;

                        case Baraban_at_0:
                            collector.motors.onIntake();
                            if (collector.colorSensor.colorState == ColorSensor.ColorSensorState.Artifact_Detected) {
                                loadArtifactInCell(0);
                                loadState = LoadState.Baraban_moving_to_05;
                            }
                            break;

                        case Baraban_moving_to_05:
                            collector.servos.setBaraban(0.25);

                            if (isRotateEnded(delayToBaraban)) {
                                loadState = LoadState.Baraban_at_05;
                            }
                            break;

                        case Baraban_at_05:
                            collector.motors.onIntake();
                            if (collector.colorSensor.colorState == ColorSensor.ColorSensorState.Artifact_Detected) {
                                loadArtifactInCell(1);
                                loadState = LoadState.Baraban_moving_to_1;
                            }
                            break;

                        case Baraban_moving_to_1:
                            collector.servos.setBaraban(0.5);

                            if (isRotateEnded(delayToBaraban)) {
                                loadState = LoadState.Baraban_at_1;
                            }
                            break;

                        case Baraban_at_1:
                            collector.motors.onIntake();
                            if (collector.colorSensor.colorState == ColorSensor.ColorSensorState.Artifact_Detected) {
                                loadArtifactInCell(2);
                                loadState = LoadState.Idle;
                            }
                            break;

                        case Idle:
                            collector.motors.reverseForAWhile(delayToReverse);

                            if (collector.motors.runTimeAll.seconds() > delayToReverse) {
                                automaticState = CollectorState.Fire;
                            }
                            break;

                        default:
                            break;
                    }

                    break;

                case Fire:
                    if(!isRandomizeWasDetected() || !isAllowFire() || !isRobotHaveMinVel()){
                        collector.motors.offFLyWheel();
                        break;}

                    switch (fireState) {
                        case Pusher_prefire:
                            collector.servos.setPusher(0.55);

                            if (collector.servos.runTimePusher.seconds() > 0.5) {
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

                            double targetPos = findNeededArtifactPos(targetColor);

                                    // Выбираем состояние движения
                            if (targetPos == 0.5) {
                                fireState = FireState.Baraban_moving_to_1;
                            } else if (targetPos == 0.25) {
                                fireState = FireState.Baraban_moving_to_05;
                            } else {
                                fireState = FireState.Baraban_moving_to_0;
                            }
                            break;

                        case Baraban_moving_to_0:
                            collector.servos.setBaraban(0.0);

                            if (isRotateEnded(delayToBaraban)) {
                                fireState = FireState.Move_angle;
                            }
                            break;

                        case Baraban_moving_to_05:
                            collector.servos.setBaraban(0.25);

                            if (isRotateEnded(delayToBaraban)) {
                                fireState = FireState.Move_angle;
                            }
                            break;

                            case Baraban_moving_to_1:
                            collector.servos.setBaraban(0.5);

                            if (isRotateEnded(delayToBaraban)) {
                                fireState = FireState.Move_angle;
                            }
                            break;
                        case Move_angle:
                            fireState = FireState.Baraban_at_pos;
//                            curAngle = getAngle(collector.servos.curAnglePos);
//                            curVel = collector.motors.curOverallVel * 0.04 * 10000 * 0.3;
//                            curLength = getLength(curAngle, curVel);
//                            curHeight = getHeight(curAngle, curVel);
//
//                            if (curLength >= range * 2 && curLength <= range * 2 + 10 && curHeight >= 105 && curHeight <= 110
//                                    && collector.servos.runTimeAngle.seconds() > delayToAngle ){
//                                fireState = FireState.Baraban_at_pos;
//                            }else {
//                                if(curLength - range * 2 <= 10){
//                                    collector.servos.setAngle(Math.max(collector.servos.curAnglePos - 0.02, 0.0));
//                                }else {
//                                    collector.servos.setAngle(Math.min(collector.servos.curAnglePos + 0.02, 0.65));
//                                }
//                            }
                            break;

                        case Baraban_at_pos:
                            if(collector.motors.curOverallVel >= curVel/6.3 *5){
                                collector.servos.setPusher(1);

                                if (collector.servos.runTimePusher.seconds() > delayToPusher) {
                                    fireState = FireState.Pusher_back;
                                }
                            }else {
                                collector.motors.setSpeed(curVel/6.3 *5);
                                break;
                            }

                            break;

                        case Pusher_back:
                            collector.servos.setPusher(0.55);

                            if (collector.servos.runTimePusher.seconds() > delayToPusher) {
                                if (collector.colorSensor.colorState == ColorSensor.ColorSensorState.No_Artifact_Detected) {
                                    deleteColorFromCell();
                                    fireState = FireState.Find_Color;
                                } else {
                                    fireState = FireState.Baraban_at_pos;
                                }
                            }
                            break;

                        case Pusher_start:
                            collector.servos.setPusher(PUSHER_START_POS);

                            if (collector.servos.runTimePusher.seconds() > 0.5) {
                                fireState = FireState.Idle;
                            }
                            break;

                        case Idle:
                            collector.motors.offFLyWheel();

                            automaticState = CollectorState.Load;
                            loadState = LoadState.Baraban_moving_to_0;
                            fireState = FireState.Find_Color;
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
    public void loadArtifactInCell(int cell){
        if(cell == 0){
            cells.cell0.table.color = collector.colorSensor.artifactColor;
            cells.cell0.table.pos = 0.0;
        } else if (cell == 1) {
            cells.cell1.table.color = collector.colorSensor.artifactColor;
            cells.cell1.table.pos = 0.25;
        } else if (cell == 2) {
            cells.cell2.table.color = collector.colorSensor.artifactColor;
            cells.cell2.table.pos = 0.5;
        }else {
            return;
        }
    }
    public void deleteColorFromCell(){
        findNeededCell().table.color = 0;
    }
    public double findNeededArtifactPos(int color){
        double currentPos = collector.servos.curBarabanPos;

        if(cells.cell2.table.color == color){
            return cells.cell2.table.pos;
        }
        if(cells.cell1.table.color == color){
            return cells.cell1.table.pos;
        }

        if(cells.cell0.table.color == color){
            return cells.cell0.table.pos;
        }


        if(cells.cell2.table.color != 0) return cells.cell2.table.pos;
        if(cells.cell1.table.color != 0) return cells.cell1.table.pos;
        if(cells.cell0.table.color != 0) return cells.cell0.table.pos;

        return currentPos;
    }

    public void checkNumberOfArtifacts(){
        int artifactNumber = 0;

        artifactNumber += cells.cell0.table.color != 0 ? 1 : 0;
        artifactNumber += cells.cell1.table.color != 0 ? 1 : 0;
        artifactNumber += cells.cell2.table.color != 0 ? 1 : 0;

        artifactCount = artifactNumber;
    }
    public Cells.Cell findNeededCell(){
        return cells.cell0.table.pos == collector.servos.curBarabanPos ? cells.cell0 : (cells.cell1.table.pos == collector.servos.curBarabanPos ? cells.cell1 : cells.cell2);
    }
    public double findNeededPosAngle(double curAngle){
        return ((90 - 65) - (90 - Math.toDegrees(curAngle))) * (185 / 23) / 270;
    }
    double getAngle(double servoPos){
        return (servoPos * 270 * 23) / 185 + 43;
    }
    double getLength(double angle, double vel){
        return (Math.sin(Math.toRadians(angle) * 2) * Math.pow(vel, 2))/ 981;
    }
    double getHeight(double angle, double vel){
        return (Math.pow(vel, 2) * Math.pow(Math.sin(Math.toRadians(angle)), 2)) / (981 * 2);
    }

    double getAngle2(double lenght){
        return Math.atan((2 * 105)/ lenght);
    }
    double getAngle3(double range){
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
    public static class Cells {
        public Cells(){
            cell0 = new Cell(0, new Cell.Table(0, 0.0));
            cell1 = new Cell(1, new Cell.Table(0, 0.0));
            cell2 = new Cell(2, new Cell.Table(0, 0.0));
        }
        private final Cell cell0, cell1, cell2;
        public static class Cell{

            public Cell(int numCell, Table table){
                this.numCell = numCell;
                this.table = table;
            }
            int numCell;
            Table table;

            public static class Table{
                public Table(int color, double pos){
                    this.color = color;
                    this.pos = pos;
                }
                double color;
                double pos;
            }
        }
    }

    @Override
    public void showData(){
        telemetry.addLine("=== AUTOMATIC ===");
        telemetry.addData("range", range);
        telemetry.addData("Speed", curVel);
        telemetry.addData("Angle", curAngle * 180 / 3.14);
        telemetry.addData("Length", curLength);
        telemetry.addData("Height", curHeight);
        telemetry.addData("Runtime", "%.1fs", timeFromLoad.seconds());
        telemetry.addData("Artifacts", artifactCount);
        telemetry.addData("Cell0", "pos:%s color:%s", cells.cell0.table.pos, getColorFromNumber(cells.cell0.table.color));
        telemetry.addData("Cell1", "pos:%s color:%s", cells.cell1.table.pos, getColorFromNumber(cells.cell1.table.color));
        telemetry.addData("Cell2", "pos:%s color:%s", cells.cell2.table.pos, getColorFromNumber(cells.cell2.table.color));
        telemetry.addData("Automatic state", automaticState.toString());
        telemetry.addData("Load state", loadState.toString());
        telemetry.addData("Fire state", fireState.toString());
        telemetry.addData("Button X", joystickActivity.buttonX);
        telemetry.addLine();
    }
    public String getColorFromNumber(double number){
        return number == 2 ? "Purple" : number == 1 ? "Green" : "Empty";
    }
}
