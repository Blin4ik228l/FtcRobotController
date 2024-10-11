package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ROBOT {
    private HardwareMap hardwareMap = null;
    private DcMotorEx rightB, rightF, leftB, leftF;
    private DcMotorEx encM, encL, encR;
    private Position deltaPosition, globalPosition;
    private double encLOld, encROld, encMOld;

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

        deltaPosition = new Position();
        globalPosition = new Position(x, y, heading);
    }

    // Этот метод должен вызываться в цикле основной программы, чтобы обновлять состояние робота
    void update(){
        updatePosition();
    }

    // Обновление положения робота на поле с помощью следящих колес
    void updatePosition (){
        double leftEncoderXnow = ticksToCm(encL.getCurrentPosition());
        double deltaLeftEncoderX = leftEncoderXnow - encLOld;
        encLOld = leftEncoderXnow;

        double rightEncoderXnow = ticksToCm(encR.getCurrentPosition());
        double deltaRightEncoderX = rightEncoderXnow - encROld;
        encROld = rightEncoderXnow;

        double encoderYnow = ticksToCm(encM.getCurrentPosition());
        double deltaEncoderY = encoderYnow - encMOld;
        encMOld = encoderYnow;

        if(leftEncoderXnow == 0 && rightEncoderXnow == 0 && encoderYnow == 0 ) {
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

    void goToPosition(Position targetpos){

    }

    void setVelocity(){

    }

    void setTeleskopePos(){

    }

    double ticksToCm(double ticks){
        return ticks / CONSTS.TICK_PER_CM;
    }
}
