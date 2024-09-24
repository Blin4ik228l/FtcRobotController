package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp
public class TeleopTest extends OpMode {

    DcMotorEx rightB, rightF, leftB, leftF;

    DcMotorEx encM, encL, encR;

    ElapsedTime runtime = new ElapsedTime();

    double Gx, Gy;
    double a = 0, x = 0;
    @Override
    public void init() {
        rightB = hardwareMap.get(DcMotorEx.class, "rightB");
        rightF = hardwareMap.get(DcMotorEx.class, "rightF");
        leftB = hardwareMap.get(DcMotorEx.class, "leftB");
        leftF = hardwareMap.get(DcMotorEx.class, "leftF");

        encM = hardwareMap.get(DcMotorEx.class, "rightF") ;
        encL = hardwareMap.get(DcMotorEx.class, "leftF") ;
        encR =  hardwareMap.get(DcMotorEx.class, "rightB");

        rightB.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightF.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftF.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftB.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        encR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        encM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        runtime.milliseconds();

        if(gamepad1.back){
            encR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            encL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            encM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            encM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            encR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            encL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        if (gamepad1.left_bumper && gamepad1.right_bumper){
            telemetry.addLine("Автоном мод ");
            double X = 0;
            double Y = 0;
            double Rx = 0;
            double Ry = 0;
            double forward = 0, side = 0, angle = 0;
            double Rad = 0;
            while(!gamepad1.a){
                if(gamepad1.b){
                    break;
                }
               while(gamepad1.left_stick_y == 0 && gamepad1.left_stick_x == 0){

                    runtime.seconds();
                    while(runtime.seconds() == 1){
                         X += gamepad1.left_stick_y * 10;
                         Y += gamepad1.left_stick_x * 10;
                    }
                   angle = gamepad1.left_stick_x * 10;
                    runtime.reset();

                    telemetry.addData("X", X);
                    telemetry.addData("y", Y);
                    telemetry.addData("Angle", angle);
                    telemetry.update();
                }

                if (rightB.getPower() + leftF.getPower() == 0 && (rightB.getPower() != 0 && leftF.getPower() != 0) ){
                    Rx = (((encL.getCurrentPosition() + encR.getCurrentPosition())/2) / CONSTS.TICK_PER_CM);
                } else  {
                    Rx = (((encL.getCurrentPosition() - encR.getCurrentPosition())/2) / CONSTS.TICK_PER_CM);
                }
                Ry = (encM.getCurrentPosition()/CONSTS.TICK_PER_CM);
                Rad = Rx/(CONSTS.DIST_BETWEEN_ENC_X/2);
                if(rightB.getPower() == 0 && leftB.getPower() == 0) {
                    Gx += Rx * Math.cos(Rad) - Rx * Math.sin(Rad);
                    Gy += Rx * Math.sin(Rad) - Rx * Math.cos(Rad);
                }

                forward = X - Gx;
                side = Y - Gy;

                rightF.setPower(Range.clip(-forward - side - angle, -1,1));
                rightB.setPower(Range.clip(-forward + side - angle, -1,1));
                leftF.setPower(Range.clip(forward - side - angle, -1,1));
                leftB.setPower(Range.clip(forward + side - angle, -1,1));


                telemetry.addData("rightB", rightB.getPower());
                telemetry.addData("rightF", rightF.getPower());
                telemetry.addData("leftB", leftB.getPower());
                telemetry.addData("leftF", leftF.getPower());
                telemetry.addLine("\"");
                telemetry.addData("Rx", Rx);
                telemetry.addData("Ry", Ry);
                telemetry.addData("Gx", Gx);
                telemetry.addData("Gy", Gy);
                telemetry.addLine("\"");
                telemetry.addData("Rad", Rad);
                telemetry.addData("angle", Math.acos(angle));
                telemetry.update();

                if(Gy - Y == 0 && Gx - X == 0){
                    angle = 0;
                    rightF.setPower(0);
                    rightB.setPower(0);
                    leftF.setPower(0);
                    leftB.setPower(0);

                    rightB.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
                    rightF.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
                    leftB.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
                    leftF.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
                }

            }
        }
        double turn  =  gamepad1.right_stick_x;
        double rightFP = Range.clip(-gamepad1.left_stick_y - gamepad1.left_stick_x - turn, -1.0, 1.0);
        double rightBP = Range.clip(-gamepad1.left_stick_y + gamepad1.left_stick_x - turn, -1.0, 1.0);
        double leftFP = Range.clip(gamepad1.left_stick_y - gamepad1.left_stick_x - turn, -1.0, 1.0);
        double leftBP = Range.clip(gamepad1.left_stick_y + gamepad1.left_stick_x - turn, -1.0, 1.0);
        double Rx = 0;
        double Ry = 0;
        double Rad = 0;
        double angle_robot = 0;

        
        rightF.setPower(rightFP);
        rightB.setPower(rightBP);
        leftF.setPower(leftFP);
        leftB.setPower(leftBP);

        if(turn == 0.0 && gamepad1.left_stick_x == 0.0 && gamepad1.left_stick_y == 0.0){
            rightF.setPower(0);
            rightB.setPower(0);
            leftF.setPower(0);
            leftB.setPower(0);

            rightB.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            rightF.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            leftB.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            leftF.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        }

        if (rightB.getPower() + leftF.getPower() != 0 ){
            Rx = (((encL.getCurrentPosition() + encR.getCurrentPosition())/2) / CONSTS.TICK_PER_CM);
            telemetry.addLine("Круговое");
            a = 1;
            x = 0;
        } else  if(rightB.getPower() < 0 && leftB.getPower() > 0 || rightB.getPower() > 0 && leftB.getPower() < 0 ){
            Rx = (((encL.getCurrentPosition() - encR.getCurrentPosition())/2) / CONSTS.TICK_PER_CM);
            telemetry.addLine("Прямое");
            x = 1;
            a = 0;
        }
        if(gamepad1.a || a == 1 ){
            Rx = (((encL.getCurrentPosition() + encR.getCurrentPosition())/2) / CONSTS.TICK_PER_CM);
            telemetry.addLine("Круговое");
        } else if (gamepad1.x || x == 1) {
            Rx = (((encL.getCurrentPosition() - encR.getCurrentPosition())/2) / CONSTS.TICK_PER_CM);
            telemetry.addLine("Прямое");
        }

        Ry = (encM.getCurrentPosition()/CONSTS.TICK_PER_CM);
        angle_robot = (Rx/CONSTS.LENGHT_ROUND_SMALL) * 360;
        Rad = Rx/(CONSTS.DIST_BETWEEN_ENC_X/2);

//        double znak = angle_robot/ Math.abs(angle_robot);
//
//        if(Math.abs(angle_robot) >= 360){
//            angle_robot = znak * (Math.abs(angle_robot) - 360);
//        }

        if(rightB.getPower() != 0 && leftB.getPower() != 0) {
            Gx += Rx * Math.cos(angle_robot) - Rx * Math.sin(angle_robot);
            Gy += Rx * Math.sin(angle_robot) + Rx * Math.cos(angle_robot);
//            Gx += Rx;
//            Gy += Ry;
        }

        if(gamepad1.b){
            telemetry.addData("Напряга в rightB", rightB.getPower());
            telemetry.addData("Напряга в rightF", rightF.getPower());
            telemetry.addData("Напряга в leftF", leftF.getPower());
            telemetry.addData("Напряга в leftB", leftB.getPower());
            telemetry.addLine("\"");
        }else {
            telemetry.addData("Левый экодер тики", encL.getCurrentPosition());
            telemetry.addData("Правый энкодер тики", encR.getCurrentPosition());
            telemetry.addData("Серединный энкодер", encM.getCurrentPosition());
            telemetry.addLine("\"");

            telemetry.addData("Левый экодер см", encL.getCurrentPosition() / (CONSTS.TICK_PER_CM));
            telemetry.addData("Правый энкодер см", encR.getCurrentPosition() / (CONSTS.TICK_PER_CM));
            telemetry.addData("Серединный энкодер см", encM.getCurrentPosition() / (CONSTS.TICK_PER_CM));
            telemetry.addLine("\"");

            telemetry.addData("Левый экодер тики/сек", encL.getVelocity());
            telemetry.addData("Правый энкодер тики/сек", encR.getVelocity());
            telemetry.addData("Серединный энкодер/сек", encM.getVelocity());
            telemetry.addLine("\"");

            telemetry.addData("Угол робота", angle_robot);
            telemetry.addData("Угол робота через рад", Math.sin(Rad));
            telemetry.addData("Радиан", Rad);
            telemetry.addLine("\"");

            telemetry.addData("Rx", Rx);
            telemetry.addData("Ry", Ry);
            telemetry.addData("Gx", Gx);
            telemetry.addData("Gy", Gy);
            telemetry.addLine("\"");
            telemetry.addData("Время", runtime.milliseconds());
        }

        telemetry.update();

        runtime.reset();
    }

}
