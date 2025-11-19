package org.firstinspires.ftc.teamcode.Modules.Players.Pl2;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Modules.MainModule;
import org.firstinspires.ftc.teamcode.Modules.Players.JoystickActivity;
import org.firstinspires.ftc.teamcode.Modules.Players.PlayerClass;
import org.firstinspires.ftc.teamcode.Robot.RobotClass;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.CollectorParts.ButtonClass;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.CollectorParts.CollectorMotors;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.CollectorParts.ColorSensor;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.CollectorParts.Servos;

public class AutomaticClass extends PlayerClass{
    public AutomaticClass(JoystickActivity joystickActivity, RobotClass.Collector collector, OpMode op) {
        super(joystickActivity, op.telemetry);
        this.collector = collector;

        cells = new Cells();
        runtime = new ElapsedTime();

        automaticState = CollectorState.Load;
        loadState = LoadState.Baraban_moving_to_0;
        fireState = FireState.Find_Color;
    }
    public RobotClass.Collector collector;
    private final Cells cells;
    private final ElapsedTime runtime;
    public int[] randomizedArtifacts = new int[3];
    public boolean isVyrCompleted;
    public double range;
    public double curVel;
    public int artifactCount;

    public void setFields(boolean isVyrCompleted, double range, double curVel){
        this.isVyrCompleted = isVyrCompleted;
        this.range = range;
        this.curVel = curVel;
    }
    public void setRandomizedArtifacts(int[] randomizedArtifacts){
        this.randomizedArtifacts = randomizedArtifacts;
    }

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
        Check_Cells;

    }
    public enum FireState{
        Baraban_moving_to_0,
        Baraban_moving_to_05,
        Baraban_moving_to_1,
        Baraban_at_pos,
        Pusher_back,
        Find_Color,
        Idle,
        Pusher_start
    }

    @Override
    public void execute(){
        if(!joystickActivity.buttonX) {
            collector.motors.targetMotorState = CollectorMotors.MotorsState.OffInTake;
        }else{
            checkNumberOfArtifacts();

            switch (automaticState){
                case Load:

                    collector.motors.targetMotorState = CollectorMotors.MotorsState.OnIntake;

                    if(collector.motors.collectorMotorsState == CollectorMotors.CollectorMotorsState.Unready) break;

                    switch (loadState) {

                        case Idle:
                            collector.motors.targetMotorState = CollectorMotors.MotorsState.ReverseForWhile;
                            if (collector.motors.runTimeAll.seconds() > 1 && collector.motors.collectorMotorsState == CollectorMotors.CollectorMotorsState.Ready) {

                                collector.motors.targetMotorState = CollectorMotors.MotorsState.OffInTake;
                                automaticState = CollectorState.Fire;
                                runtime.reset();
                            }
                            break;

                        case Baraban_at_1:
                            if (collector.colorSensor.colorState == ColorSensor.ColorSensorState.Artifact_Detected) {
                                loadArtifactInCell(2);
                                loadState = LoadState.Idle;
                                runtime.reset();
                            }
                            break;

                        case Baraban_moving_to_1:
                            collector.servos.barabanPos = 1;
                            collector.servos.execute();
                            if ( (collector.servos.runTimeBaraban.seconds() > 0.6 && collector.servos.barabanState == Servos.BarabanState.inPos)) {
                                collector.buttonClass.buttonClassState = ButtonClass.ButtonClassState.upDateState;
                                loadState = LoadState.Baraban_at_1;
                            }
                            break;

                        case Baraban_at_05:
                            if (collector.colorSensor.colorState == ColorSensor.ColorSensorState.Artifact_Detected) {
                                loadArtifactInCell(1);
                                loadState = LoadState.Baraban_moving_to_1;
                            }
                            break;


                        case Baraban_moving_to_05:
                            collector.servos.barabanPos = 0.5;
                            collector.servos.execute();
                            if ( (collector.servos.runTimeBaraban.seconds() > 0.6 && collector.servos.barabanState == Servos.BarabanState.inPos)) {
                                collector.buttonClass.buttonClassState = ButtonClass.ButtonClassState.upDateState;
                                loadState = LoadState.Baraban_at_05;
                            }
                            break;

                        case Baraban_at_0:
                            if (collector.colorSensor.colorState == ColorSensor.ColorSensorState.Artifact_Detected) {
                                loadArtifactInCell(0);
                                loadState = LoadState.Baraban_moving_to_05;
                            }
                            break;

                        case Baraban_moving_to_0:
                            collector.servos.barabanPos = 0.0;
                            collector.servos.execute();
                            if ((collector.servos.runTimeBaraban.seconds() > 0.6 && collector.servos.barabanState == Servos.BarabanState.inPos)) {
                                collector.buttonClass.buttonClassState = ButtonClass.ButtonClassState.upDateState;
                                loadState = LoadState.Baraban_at_0;
                            }
                            break;

                        default:
                            break;
                    }

                    break;

                case Fire:

                    collector.motors.targetMotorState = CollectorMotors.MotorsState.OffFlyWheel;
                    if(!isRandomizeWasDetected() || !isAllowFire() || !isRobotHaveMinVel()) break;

                    collector.motors.targetMotorState = CollectorMotors.MotorsState.OnFlyWheel;
                    if (collector.motors.collectorMotorsState == CollectorMotors.CollectorMotorsState.Unready
                            || (Math.abs(collector.motors.curLeftVel) < 4 && Math.abs(collector.motors.curRightVel) < 4)) break;

                    switch (fireState) {
                        case Idle:
                            collector.motors.targetMotorState = CollectorMotors.MotorsState.OffInTake;
                            if (collector.motors.collectorMotorsState == CollectorMotors.CollectorMotorsState.Ready) {
                                loadState = LoadState.Baraban_moving_to_0;
                                fireState = FireState.Find_Color;
                                automaticState = CollectorState.Load;
                                joystickActivity.buttonBack = false;
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
                            if (targetPos == 1.0) {
                                fireState = FireState.Baraban_moving_to_1;
                            } else if (targetPos == 0.5) {
                                fireState = FireState.Baraban_moving_to_05;
                            } else {
                                fireState = FireState.Baraban_moving_to_0;
                            }

                            break;
                        case Baraban_moving_to_0:
                            collector.servos.barabanPos = 0.0;
                            collector.servos.execute();
                            if ((collector.servos.runTimeBaraban.seconds() > 0.6 && collector.servos.barabanState == Servos.BarabanState.inPos)) {
                                collector.buttonClass.buttonClassState = ButtonClass.ButtonClassState.upDateState;
                                fireState = FireState.Baraban_at_pos;
                            }
                            break;
                        case Baraban_moving_to_05:
                            collector.servos.barabanPos = 0.5;
                            collector.servos.execute();
                            if ((collector.servos.runTimeBaraban.seconds() > 0.6 && collector.servos.barabanState == Servos.BarabanState.inPos)) {
                                collector.buttonClass.buttonClassState = ButtonClass.ButtonClassState.upDateState;
                                fireState = FireState.Baraban_at_pos;
                            }
                            break;

                            case Baraban_moving_to_1:
                            collector.servos.barabanPos = 1;
                            collector.servos.execute();
                            if ((collector.servos.runTimeBaraban.seconds() > 0.6 && collector.servos.barabanState == Servos.BarabanState.inPos)) {
                                collector.buttonClass.buttonClassState = ButtonClass.ButtonClassState.upDateState;
                                fireState = FireState.Baraban_at_pos;
                            }
                            break;

                        case Baraban_at_pos:
                            collector.servos.pusherPos = 0.9;
                            collector.servos.execute();
                            if (collector.servos.runTimePusher.seconds() > 0.8 && collector.servos.pusherState == Servos.PusherState.inPos) {
                                fireState = FireState.Pusher_back;
                            }
                            break;

                        case Pusher_back:
                            collector.servos.pusherPos = 0.45;
                            collector.servos.execute();

                            if (collector.servos.runTimePusher.seconds() > 0.8 && collector.servos.pusherState == Servos.PusherState.inPos) {
                                if (collector.colorSensor.colorState == ColorSensor.ColorSensorState.No_Artifact_Detected) {
                                    deleteColorFromCell();
                                    fireState = FireState.Find_Color;
                                } else {
                                    fireState = FireState.Baraban_at_pos;
                                }
                            }
                            break;
                        case Pusher_start:
                            collector.servos.pusherPos = PUSHER_START_POS;
                            collector.servos.execute();
                            if (collector.servos.runTimePusher.seconds() > 0.8 && collector.servos.pusherState == Servos.PusherState.inPos) {
                                fireState = FireState.Idle;
                            }
                            break;
                        default:
                            break;
                    }

                    break;
                default:
                    throw new IllegalStateException("Unexpected value: " + automaticState);
            }
        }
        collector.servos.execute();
        collector.motors.execute();
    }

    public void loadArtifactInCell(int cell){
        if(cell == 0){
            cells.cell0.table.color = collector.colorSensor.artifactColor;
            cells.cell0.table.pos = 0.0;
        } else if (cell == 1) {
            cells.cell1.table.color = collector.colorSensor.artifactColor;
            cells.cell1.table.pos = 0.5;
        } else if (cell == 2) {
            cells.cell2.table.color = collector.colorSensor.artifactColor;
            cells.cell2.table.pos = 1.0;
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
        return cells.cell0.table.pos == collector.servos.getBaraban().getPosition() ? cells.cell0 : (cells.cell1.table.pos == collector.servos.getBaraban().getPosition() ? cells.cell1 : cells.cell2);
    }
    public double findNeededPosAngle(double length){
        double neededPos = 0;

//        if(getHeight(getAngle(length)) >= 94){
//            return getAngle(length)/Math.toRadians(30);
//        }

        return collector.servos.getAngle().getPosition();
    }
    double getAngle(double length){
        double velBall = 450;

        return (length * 981) / (Math.pow(velBall, 2)) / 2.0;
    }
    double getHeight(double angle){
        double velBall = 450;

        return (Math.pow(velBall, 2) * Math.pow(Math.sin(angle), 2)) / (981 * 2);
    }

    public boolean isRobotHaveMinVel(){
        return Math.abs(curVel) < 25;
    }
    public boolean isAllowFire(){
        return isVyrCompleted && joystickActivity.buttonBack;
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
        telemetry.addData("Runtime", "%.1fs", runtime.seconds());
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
