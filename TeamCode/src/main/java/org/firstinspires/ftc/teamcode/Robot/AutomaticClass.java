package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Modules.Module;
import org.firstinspires.ftc.teamcode.Modules.Players.Pl1.Player1;
import org.firstinspires.ftc.teamcode.Modules.Players.Pl2.Player2;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DrivetrainParts.Odometry.Parts.MathUtils.Vector2;

import java.util.Date;

public class AutomaticClass extends Module {
    public AutomaticClass(Player1 player1, Player2 player2, OpMode op) {
        super(op.telemetry);

        cells = new Cells();
        runtime = new ElapsedTime();

        this.player1 = player1;
        this.player2 = player2;

        drivetrain = player1.driveTrain;
        collector = player2.collector;
    }
    public Player1 player1;
    public Player2 player2;
    public RobotClass.MecanumDrivetrain drivetrain;
    public RobotClass.Collector collector;
    private double loadedArtifactColor;
    private double loadedArtifactColor2;
    private double lastArtifactColor;
    private double curDistance;
    private double curDistance2;
    private final Cells cells;
    private final ElapsedTime runtime;
    public boolean isWhileFiring = false;
    public boolean isAllowFire = false;
    public boolean isButtonY = false;
    public boolean isArtifactsWasDetected = false;
    public double barabanPos = BARABAN_START_POS;
    public double lastBarabanPos;
    public double pusherPos = PUSHER_START_POS;
    public double lastPusherPos;
    public double anglePos = ANGLE_START_POS;
    public double lastAnglePos;
    public double inTakePower;
    public int radianSpeed;
    public double curRange;
    public double curRadians;

    public void execute(){
//        update();
        calculate();
//        action();
    }
    public void update(){
        isArtifactsWasDetected = drivetrain.exOdometry.camera.randomizedArtifact[0] != 0;
        curRange = drivetrain.exOdometry.getRange();

        collector.colorSensor.update();

        loadedArtifactColor = collector.colorSensor.currentArtifact;
        loadedArtifactColor2 = collector.colorSensor.currentArtifact1;
//        lastArtifactColor = collector.colorSensor.lastSeenArtifact;
        curDistance = collector.colorSensor.curDistance;
        curDistance2 = collector.colorSensor.curDistance1;
    }
    public void calculate(){
        if(!isWhileFiring){
            switch (checkNumberOfArtifacts()){
                case 0:
                    action();
                    if (runtime.seconds() < 0.5) return;

                    if(!player1.joystickActivity.buttonB) {
                        inTakePower = 0;
                        radianSpeed = 0;
                        return;}

                    barabanPos = 0;
                    inTakePower = -1;
                    radianSpeed = -1;
                    action();

                    if(runtime.seconds() < 1) return;
                    //Если по датчику дистанци меньше 10 см, то значит что - то, есть, смотрим на переменую которая обновляется постояно,
                    // если по не ноль записываем текущий цвет,
                    // иначе смотрим на переменную, которая обновилась единажды и там зафиксировался цвет при пролёте.
                    // if(curDistance < 10){
                    //   if(loadedArtifactColor != 0) return loadedArtifactColor;
                    //   else if(lastArtifactColor != 0) return lastArtifactColor;
                    //   else return 0;
                    // }
                    action();
                    update();
                    if(curDistance < 10 || curDistance2 < 10){
                        if(loadedArtifactColor != 0) {
                            cells.cell0.table.color = loadedArtifactColor;
                            cells.cell0.table.pos = barabanPos;
                            runtime.reset();
                            break;}
                        else {
                            if(loadedArtifactColor2 != 0)  {
                                cells.cell0.table.color = loadedArtifactColor2;
                                cells.cell0.table.pos = barabanPos;
                                runtime.reset();
                                break;}
                            else {cells.cell0.table.color = 0;}
                        }
                    }else {cells.cell0.table.color = 0;}

                    cells.cell0.table.pos = barabanPos;
                    action();
                    runtime.reset();
                    break;
                case 1:
                    action();
                    if (runtime.seconds()< 0.5) return;
                    if(!player1.joystickActivity.buttonB) {
                        inTakePower = 0;
                        radianSpeed = 0;
                        return;}
                    inTakePower = -1;
                    radianSpeed = -1;
                    barabanPos = 0.5;
                    action();

                    if(runtime.seconds() < 1) return;
                    update();
                    action();
                    if(curDistance < 10 || curDistance2 < 10){
                        if(loadedArtifactColor != 0) {
                            cells.cell1.table.color = loadedArtifactColor;
                            cells.cell1.table.pos = barabanPos;
                            runtime.reset();
                            break;}
                        else {
                            if(loadedArtifactColor2 != 0)  {
                                cells.cell1.table.color = loadedArtifactColor2;
                                cells.cell1.table.pos = barabanPos;
                                runtime.reset();
                                break;}
                            else {cells.cell1.table.color = 0;}
                        }
                    }else {cells.cell1.table.color = 0;}

                    cells.cell1.table.pos = barabanPos;
                    action();

                    runtime.reset();
                    break;
                case 2:
                    action();
                    if(runtime.seconds() < 0.5) return;

                    if(!player1.joystickActivity.buttonB) {
                        inTakePower = 0;
                        radianSpeed = 0;
                        return;}

                    inTakePower = -1;
                    radianSpeed = -1;
                    barabanPos = 1;
                    action();

                    if(runtime.seconds() < 1) return;

                    update();
                    if(curDistance < 10 || curDistance2 < 10){
                        if(loadedArtifactColor != 0) {
                            cells.cell2.table.color = loadedArtifactColor;
                            cells.cell2.table.pos = barabanPos;
                            runtime.reset();
                            break;}
                        else {
                            if(loadedArtifactColor2 != 0) {
                                cells.cell2.table.color = loadedArtifactColor2;
                                cells.cell2.table.pos = barabanPos;
                                runtime.reset();
                                break;}
                            else {cells.cell2.table.color = 0;}
                        }
                    }else {cells.cell2.table.color = 0;}

                    cells.cell2.table.pos = barabanPos;

                    action();
                    runtime.reset();
                    break;
                case 3:
                    action();
                    if(runtime.seconds() < 0.5) return;

                    inTakePower = 1;
                    radianSpeed = 0;
                    action();

                    if(runtime.seconds() < 1.5) return;
                    inTakePower = 0;
                    action();
                    isWhileFiring = true;
                    runtime.reset();
                    break;
            }

        }else{
            if(isArtifactsWasDetected) {
                switch (checkNumberOfArtifacts()) {
                    case 3:
                        action();
                        if(runtime.seconds() < 0.8) return;

                        if(!player1.joystickActivity.buttonB) {
                            inTakePower = 0;
                            radianSpeed = 0;
                            return;}

                        pusherPos = 0.45;
                        anglePos = findNeededPosAngle(curRange);

                        action();
                        if (runtime.seconds() < 1.5) return;

                        radianSpeed = 5;
                        barabanPos = findNeededArtifactPos(drivetrain.exOdometry.camera.randomizedArtifact[0]);
                        action();

                        if (runtime.seconds() < 2.5) return;

                        update();
                        if(curDistance < 10 || curDistance2 < 10){
                            if(loadedArtifactColor != 0)findNeededCell().table.color = loadedArtifactColor;
                            else {
                                if(loadedArtifactColor2 != 0) findNeededCell().table.color = loadedArtifactColor2;
                                else {
                                    findNeededCell().table.color = 0;
                                    runtime.reset();
                                    break;}
                            }
                        }else {
                            findNeededCell().table.color = 0;
                            runtime.reset();
                            break;
                        }

                        if(!(isRobotHaveMinVel() && isAllowFire() && isButtonY())) {
                            return;}

                        pusherPos = 0.9;
                        action();

                        runtime.reset();
                        break;
                    case 2:
                        action();
                        if(runtime.seconds() < 0.8) return;

                        if(!player1.joystickActivity.buttonB) {
                            inTakePower = 0;
                            radianSpeed = 0;
                            break;}

                        pusherPos = 0.45;
                        anglePos = findNeededPosAngle(curRange);

                        action();
                        if (runtime.seconds() < 1.5) return;

                        radianSpeed = 5;
                        barabanPos = findNeededArtifactPos(drivetrain.exOdometry.camera.randomizedArtifact[1]);
                        action();

                        if (runtime.seconds() < 2.5 ) return;

                        update();
                        if(curDistance < 10 || curDistance2 < 10){
                            if(loadedArtifactColor != 0)findNeededCell().table.color = loadedArtifactColor;
                            else {
                                if(loadedArtifactColor2 != 0) findNeededCell().table.color = loadedArtifactColor2;
                                else {
                                    findNeededCell().table.color = 0;
                                    runtime.reset();
                                    break;}
                            }
                        }else {
                            findNeededCell().table.color = 0;
                            runtime.reset();
                            break;}

                        if(!(isRobotHaveMinVel() && isAllowFire() && isButtonY())) {
                            return;}

                        pusherPos = 0.9;
                        action();

                        runtime.reset();
                        break;
                    case 1:
                        action();
                        if(runtime.seconds() < 0.8) return;

                        if(!player1.joystickActivity.buttonB) {
                            inTakePower = 0;
                            radianSpeed = 0;
                            break;}

                        pusherPos = 0.45;
                        anglePos = findNeededPosAngle(curRange);
                        action();

                        if (runtime.seconds() < 1.5) return;
                        radianSpeed = 5;
                        barabanPos = findNeededArtifactPos(drivetrain.exOdometry.camera.randomizedArtifact[2]);
                        action();

                        if (runtime.seconds() < 2.5) return;
                        update();
                        if(curDistance < 10 || curDistance2 < 10){
                            if(loadedArtifactColor != 0)findNeededCell().table.color = loadedArtifactColor;
                            else {
                                if(loadedArtifactColor2 != 0) findNeededCell().table.color = loadedArtifactColor2;
                                else {
                                    findNeededCell().table.color = 0;
                                    runtime.reset();
                                    break;}
                            }
                        }else {
                            findNeededCell().table.color = 0;
                            runtime.reset();
                            break;}

                        if(!(isRobotHaveMinVel() && isAllowFire() && isButtonY())){
                            runtime.reset();
                        return;}

                        pusherPos = 0.9;
                        action();

                        runtime.reset();
                        break;
                    case 0:
                        action();
                        if(runtime.seconds() < 0.8) return;

                        radianSpeed = 0;
                        pusherPos = PUSHER_START_POS;
                        action();

                        if(runtime.seconds() < 1.5) break;

                        isWhileFiring = false;
                        runtime.reset();
                        break;
                }
            }
        }
    }

    public int checkNumberOfArtifacts(){
        int artifactNumber = 0;

        artifactNumber += cells.cell0.table.color != 0 ? 1 : 0;
        artifactNumber += cells.cell1.table.color != 0 ? 1 : 0;
        artifactNumber += cells.cell2.table.color != 0 ? 1 : 0;

        return artifactNumber;
    }

    public double findNeededArtifactPos(int color){
        if(barabanPos == cells.cell0.table.pos) return cells.cell0.table.color == color ? cells.cell0.table.pos  : (cells.cell1.table.color == color ? cells.cell1.table.pos : (cells.cell2.table.color == color ?  cells.cell2.table.pos: barabanPos));
        else if (barabanPos == cells.cell1.table.pos ) return cells.cell1.table.color == color ? cells.cell1.table.pos  : (cells.cell0.table.color == color ? cells.cell0.table.pos : (cells.cell2.table.color == color ?  cells.cell2.table.pos: barabanPos));
        else return cells.cell2.table.color == color ? cells.cell2.table.pos  : (cells.cell1.table.color == color ? cells.cell1.table.pos : (cells.cell0.table.color == color ?  cells.cell0.table.pos: barabanPos));
    }
    public Cells.Cell findNeededCell(){
        return cells.cell0.table.pos == barabanPos ? cells.cell0 : (cells.cell1.table.pos == barabanPos ? cells.cell1 : cells.cell2);
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
        return drivetrain.exOdometry.robotSelfCentricVel.length() < 25;
    }
    public double getCurFlyWheelVel(){
        return collector.encoders.getVelocity();
    }
    public boolean isButtonY(){
        return player1.joystickActivity.buttonY;
    }
    public boolean isAllowFire(){
        return Math.abs(drivetrain.exOdometry.getDeltaAngle(drivetrain.exOdometry.getFoundedRobotAngle())) < Math.abs(2);
    }
    public void action(){

        collector.setPowerAndPos(inTakePower, radianSpeed, barabanPos, pusherPos, anglePos);
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
    public void showData(){
        telemetry.addLine("Automatic class caption")
                .addData("Time in seconds", "%.1f %n", runtime.seconds())
                .addData("Artifacts number", checkNumberOfArtifacts())
                .addData("cell0", "pos[%s] color[%s]%n", cells.cell0.table.pos, getColorFromNumber(cells.cell0.table.color))
                .addData("cell1", "pos[%s] color[%s]%n", cells.cell1.table.pos, getColorFromNumber(cells.cell1.table.color))
                .addData("cell2", "pos[%s] color[%s]%n", cells.cell2.table.pos, getColorFromNumber(cells.cell2.table.color));
        telemetry.addLine();
    }
    public String getColorFromNumber(double number){
        return number == 2 ? "Purple" : number == 1 ? "Green" : "Empty";
    }
}
