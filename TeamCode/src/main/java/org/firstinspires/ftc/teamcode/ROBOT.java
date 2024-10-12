package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Deque;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Objects;
import java.util.Queue;
import java.util.Stack;

public class ROBOT {
    private HardwareMap hardwareMap = null;
    private DcMotorEx rightB, rightF, leftB, leftF;
    private DcMotorEx encM, encL, encR;
    private double encLOld, encROld, encMOld;
    private Thread threadOdometry;
    private Deque<Task> taskDeQueue;
    Odometry odometry;
    private Stack<Task> CompletedTask;
    private PID pid;

    public void executeTask(){
        Deque<Task> executingTask = null;
        while(!taskDeQueue.isEmpty()){
            switch (taskDeQueue.getFirst().runMode){
                case START_AFTER_PREVIOUS:
                    assert executingTask != null;
                    if(executingTask.isEmpty()){
                        executingTask.addLast(taskDeQueue.getFirst());
                        taskDeQueue.removeFirst();
                    }
                    break;
                case START_WITH_PREVIOUS:
                    assert executingTask != null;
                    executingTask.addLast(taskDeQueue.getFirst());
                    taskDeQueue.removeFirst();
                    break;
                case HOTCAKE:
                    assert executingTask != null;
                    executingTask.addFirst(taskDeQueue.getFirst());
                    taskDeQueue.removeFirst();
                    break;
            }
            Iterator<Task> task = executingTask.iterator();
            while ( task.hasNext() ) {
                Task currentTask = task.next();
                int result;
                switch (currentTask.taskType){
                    case SET_TELESKOPE:
                        result = setTeleskopePos(task.next().args[0], task.next().args[1]);
                        break;
                    case DRIVE_TO_POSITION:
                        Position pos = new Position(task.next().args[0], task.next().args[1], task.next().args[2]);
                        result = driveToPosition(pos);
                        break;
                    default:
                        result = -1;
                        break;
                }
                switch (result){
                    case -1:
                        break;
                    case 0:
                        CompletedTask.push(currentTask);
                        task.remove();
                }
                if (currentTask.runMode == Task.runModeList.HOTCAKE) {break;}
            }
        }
    }

    public static class Task{
        Task(runModeList runMode, taskTypeList taskType, double[] args){
            this.runMode = runMode;
            this.taskType = taskType;
            this.args = args;
        }

        double []args;
        runModeList runMode;
        taskTypeList taskType;

        enum taskTypeList{
            DRIVE_TO_POSITION,
            SET_TELESKOPE,
            STUCK_WHILE_DRIVING,
            STUCK_WHILE_UPPING_TELE
        }

        enum runModeList{
            START_AFTER_PREVIOUS,
            START_WITH_PREVIOUS,
            HOTCAKE
        }
    }

    private void addTask(Task newtask){
        taskDeQueue.addLast(newtask);
    }
    private static class Odometry extends Thread{
        Odometry (double x, double y, double heading, DcMotorEx encL, DcMotorEx encR, DcMotorEx encM){
            this.globalPosition.x = x;
            this.globalPosition.y = y;
            this.globalPosition.heading = heading;
            this.encL = encL;
            this.encR = encR;
            this.encM = encM;
        }

        private double encLOld, encROld, encMOld;
        private final DcMotorEx encM, encL, encR;
        private Position deltaPosition, globalPosition;

        @Override
        public void run() {
            while (isAlive()){
                updatePosition();
            }
        }

        private double ticksToCm(double ticks){
            return ticks / CONSTS.TICK_PER_CM;
        }

        synchronized Position getGlobalPosition(){
            return globalPosition;
        }
        synchronized Position getDeltaPosition(){
            return deltaPosition;
        }
        // Обновление положения робота на поле с помощью следящих колес
        private void updatePosition (){
            double leftEncoderXNow = ticksToCm(encL.getCurrentPosition());
            double deltaLeftEncoderX = leftEncoderXNow - encLOld;
            encLOld = leftEncoderXNow;

            double rightEncoderXnow = ticksToCm(encR.getCurrentPosition());
            double deltaRightEncoderX = rightEncoderXnow - encROld;
            encROld = rightEncoderXnow;

            double encoderYnow = ticksToCm(encM.getCurrentPosition());
            double deltaEncoderY = encoderYnow - encMOld;
            encMOld = encoderYnow;

            if(leftEncoderXNow == 0 && rightEncoderXnow == 0 && encoderYnow == 0 ) {
                return;
            }

            double deltaRad = (deltaRightEncoderX + deltaLeftEncoderX)/CONSTS.DIST_BETWEEN_WHEEL_X;
            double deltaX = ticksToCm(deltaLeftEncoderX + deltaRightEncoderX) / 2.0;
            double deltaY = ticksToCm(deltaEncoderY) - deltaRad * CONSTS.OFFSET_ENC_M_FROM_CENTER;

            deltaPosition.x = deltaX;
            deltaPosition.y = deltaY;
            deltaPosition.heading = deltaRad;

            globalPosition.x += deltaX * Math.cos(globalPosition.heading) - deltaY * Math.sin(globalPosition.heading);
            globalPosition.y += deltaX * Math.sin(globalPosition.heading) + deltaY * Math.cos(globalPosition.heading);
            globalPosition.heading += deltaRad;

            if (Math.abs(globalPosition.heading) >= 2 * Math.PI) {
                globalPosition.heading %= 2 * Math.PI;
            }
        }

    }

    private static class Position{
        double x;
        double y;
        double heading;

        Position(double x ,double y, double heading){
            this.x = x;
            this.y = y;
            this.heading = heading;

        }
        Position(){
            this.x = 0;
            this.y = 0;
            this.heading = 0;
        }
    }

    void init(double x, double y, double heading){
        rightB = hardwareMap.get(DcMotorEx.class, "rightB");
        rightF = hardwareMap.get(DcMotorEx.class, "rightF");
        leftB = hardwareMap.get(DcMotorEx.class, "leftB");
        leftF = hardwareMap.get(DcMotorEx.class, "leftF");

        encM = hardwareMap.get(DcMotorEx.class, "encM") ;
        encL = hardwareMap.get(DcMotorEx.class, "encL") ;
        encR =  hardwareMap.get(DcMotorEx.class, "encR");

        rightB.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightF.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftF.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftB.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        encR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        odometry = new Odometry(x, y, heading, encL,  encR, encM);
        odometry.run();
    }

    // Этот метод должен вызываться в цикле основной программы, чтобы обновлять состояние робота
    void update(){
        odometry.updatePosition();
    }

    private int driveToPosition(Position position){
        //Найти угол между heading робота и направлением куда ехать

        int result = -1;
        return result;
    }

    void setVelocity(){

    }

    int setTeleskopePos(double x, double speed){
        //если робот вдруг поехал
        //если телескоп не поднялся на нужный уровень и стоит на месте долго

        int result = -1;
        return result;
    }

    double ticksToCm(double ticks){
        return ticks / CONSTS.TICK_PER_CM;
    }

}

