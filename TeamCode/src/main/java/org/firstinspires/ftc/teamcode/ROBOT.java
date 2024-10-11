package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareMap;
public class ROBOT {
    HardwareMap hardwareMap = null;

    DcMotorEx rightB, rightF, leftB, leftF;

    DcMotorEx encM, encL, encR;

    ROBOT robot = new ROBOT();

    Position deltaPosition, globalPosition;

    double encLOld, encROld, encMOld, Rad = 0;

    class Position{
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

    double ticksToCm(double ticks){
        return ticks/CONSTS.TICK_PER_CM;
    }


    void robotCordUpd (){
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
            return;}

        double deltaRad = (deltaRightEncoderX + deltaLeftEncoderX)/CONSTS.DIST_BETWEEN_WHEEL_X;
        double deltaX = ticksToCm(deltaLeftEncoderX + deltaRightEncoderX) / 2.0;
        double deltaY = ticksToCm(deltaEncoderY) - deltaRad * CONSTS.OFFSET_ENC_M_FROM_CENTER;

        deltaPosition.x = deltaX;
        deltaPosition.y = deltaY;
        deltaPosition.heading = deltaRad;

        globalPosition.x += deltaX * Math.cos(Rad) - deltaY * Math.sin(Rad);
        globalPosition.y += deltaX * Math.sin(Rad) + deltaY * Math.cos(Rad);
        globalPosition.heading += deltaRad;

        if (Math.abs(globalPosition.heading) >= 2 * Math.PI) {
            globalPosition.heading %= 2 * Math.PI;
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
        globalPosition = new Position(x, y ,heading);
    }

}
