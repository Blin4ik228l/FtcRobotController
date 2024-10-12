package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.TimestampedI2cData;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp
public class TeleopTest extends OpMode {
    DcMotorEx rightB, rightF, leftB, leftF;

    DcMotorEx encM, encL, encR;

    ROBOT robot = new ROBOT();

    ElapsedTime runtime = new ElapsedTime();
    ElapsedTime runtime2 = new ElapsedTime();

    double Gx, Gy, currAngle;
    double a = 0, x = 0;
    int rounds = 0;
    double targetVelAngle, targetVelAngleY, targetVelAngleX, btimes = 0, timer,
            velAngleDeltaY, velAngleDelta, velAngleDeltaX, velocityX,velocityY, velocityAngle, targetAngle, targetX, targetY;

    double posXNow, posYNow, posXRotNow, deltaPosXNow, deltaPosYNow, deltaPosXRotNow, posXMin, posYMin, posXRotMin, posXMax, posYMax, posXRotMax;

    @Override
    public void init() {
        robot.init(new Position(0,0,0));
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

        double Rx = 0;
        double Razn = 0;
        double Ry = 0;

        double Rad = 0;
        double angle_robot = 0;

//        double velocityAngle = (((encL.getVelocity() + encR.getVelocity())/2)/CONSTS.TICK_PER_CM)/CONSTS.DIST_BETWEEN_ENC_X/2;// /сек
//        double velocityX = ((encL.getVelocity() - encR.getVelocity())/2)/CONSTS.TICK_PER_CM;// см/сек
//        double velocityY = (encM.getVelocity()/CONSTS.TICK_PER_CM) - velocityAngle * CONSTS.OFFSET_ENC_M_FROM_CENTER;// см/сек
//
//         double targetVelX = ((gamepad1.left_stick_y * CONSTS.MAX_TPS_ENCODER)/CONSTS.TICK_PER_CM) ;// см/сек
//          double targetVelY = ((gamepad1.left_stick_x * CONSTS.MAX_TPS_ENCODER)/CONSTS.TICK_PER_CM);// см/сек
//         targetVelAngle += ((turn * CONSTS.MAX_TPS_ENCODER/CONSTS.TICK_PER_CM)/CONSTS.DIST_BETWEEN_ENC_X/2) - velocityAngle;// // сек
//
//        double forward = (targetVelX - velocityX) * kP + targetVelX * kF;
//        double side = (targetVelY - velocityY) * kP + targetVelY * kF;
//        double angle = (targetVelAngle - velAngleDelta) * kP + targetVelAngle * kFR;
//        double kF = 110/CONSTS.MAX_TPS_ENCODER;//макс см/сек
//        double kP = -0.0001;
//        double speed = 0.8;
//        double kFR = speed/CONSTS.MAX_RAD_PER_SEC;//макс рад/сек
//        double kV = 1;
//        double posXNow = gamepad1.left_stick_x;
//        double posXMax = 0, posXMin = 0;
//        double posYNow = gamepad1.left_stick_y;
//        double posYMax = 0, posYMin = 0;
//        double rightFP = Range.clip(-gamepad1.left_stick_y - gamepad1.left_stick_x - turn, -1.0, 1.0);
//        double rightBP = Range.clip(-gamepad1.left_stick_y + gamepad1.left_stick_x - turn, -1.0, 1.0);
//        double leftFP = Range.clip(gamepad1.left_stick_y - gamepad1.left_stick_x - turn, -1.0, 1.0);
//        double leftBP = Range.clip(gamepad1.left_stick_y + gamepad1.left_stick_x - turn, -1.0, 1.0);
//        double posXNow = gamepad1.left_stick_x;
//       double posXMax = 0, posXMin = 0;
//       double posYNow = gamepad1.left_stick_y;
//       double posYMax = 0, posYMin = 0;


//        if(posYNow > posYMax){
//            posYMax = posYNow;
//        } else if (posYNow < posYMin) {
//            posYMin = posYNow;
//        }
//
//        if(posYNow > posYMin && posYNow < posYMax){
//            posYNow = 0;
//        }
//
//        if(posXNow > posXMax){
//            posXMax = posXNow;
//        } else if (posXNow < posXMin) {
//            posXMin = posXNow;
//        }
//
//        if(posXNow > posXMin && posXNow < posXMax){
//            posXNow = 0;
//        }
//        if(velAngleDeltaY >= CONSTS.MAX_RAD_PER_SEC){
//            velAngleDeltaY = CONSTS.MAX_RAD_PER_SEC;
//        }else if(velAngleDeltaY <= -CONSTS.MAX_RAD_PER_SEC){
//            velAngleDeltaY = -CONSTS.MAX_RAD_PER_SEC;
//        }
//
//        if(velAngleDeltaX >= CONSTS.MAX_RAD_PER_SEC ){
//            velAngleDeltaX = CONSTS.MAX_RAD_PER_SEC;
//        }else if(velAngleDeltaX <= -CONSTS.MAX_RAD_PER_SEC){
//            velAngleDeltaX = -CONSTS.MAX_RAD_PER_SEC;
//        }
//
//        if(velAngleDelta >= CONSTS.MAX_RAD_PER_SEC){
//            velAngleDelta = CONSTS.MAX_RAD_PER_SEC;
//        }else if(velAngleDelta <= -CONSTS.MAX_RAD_PER_SEC){
//            velAngleDelta = -CONSTS.MAX_RAD_PER_SEC;
//        }



        if(rightB.getPower() != 0 && leftB.getPower() != 0) {

        }

        currAngle = Math.acos(Gy/Math.sqrt(((Gx * Gx) + (Gy*Gy))));

        double currentAngle = (((double) (encL.getCurrentPosition() + encR.getCurrentPosition()) /2) / CONSTS.TICK_PER_CM)/ (CONSTS.DIST_BETWEEN_WHEEL_X/2);// рад/сек
        double currentX = (((double) (encL.getCurrentPosition() - encR.getCurrentPosition()) /2) / CONSTS.TICK_PER_CM);
        double currentY = (encM.getCurrentPosition()/CONSTS.TICK_PER_CM);// рад/сек

        targetY += (-gamepad1.left_stick_x * CONSTS.DIST_FIELD_Y);
        targetX += (-gamepad1.left_stick_y * CONSTS.DIST_FIELD_X);
        targetAngle = (gamepad1.right_stick_x * CONSTS.LENGTH_ROUND_BIG) / (CONSTS.DIAM_CIRCLE_ROBOT/2);

        double speed = 1;
        double kF = speed/CONSTS.MAX_DIST;//макс см/сек
        double kP = -0.004;
        double kFR = speed/CONSTS.MAX_RAD;//макс рад/сек

//        velAngleDeltaY -= (-posXNow * CONSTS.MAX_TPS_ENCODER/CONSTS.TICK_PER_CM)/(CONSTS.DIST_BETWEEN_ENC_X/2)- velocityY;// рад/сек
//        velAngleDeltaX -= (-posYNow * CONSTS.MAX_TPS_ENCODER/CONSTS.TICK_PER_CM)/(CONSTS.DIST_BETWEEN_ENC_X/2)- velocityX;// рад/сек
//        velAngleDelta += (gamepad1.right_stick_x * CONSTS.MAX_TPS_ENCODER/CONSTS.TICK_PER_CM)/(CONSTS.DIST_BETWEEN_ENC_X/2) - velocityAngle;// рад/сек

//        double forward = ( targetX - Rx ) * kP + targetX* kF;
//        double side = (targetY - Ry) * kP + targetY * kF;
//        double angle = (targetAngle - Rad ) * kP + targetAngle* kFR;
        double forward = (targetX - Gx) * kP + targetX * kF;
        double side = (targetY - Gy) * kP + targetY * kF;
        double angle = (targetAngle - Rad) * kP + targetAngle * kFR;

        double rightFP = Range.clip((-forward - side - angle), -1.0, 1.0);
        double leftBP = Range.clip((forward + side - angle), -1.0, 1.0);
        double leftFP = Range.clip((forward - side - angle), -1.0, 1.0);
        double rightBP = Range.clip((-forward + side - angle), -1.0, 1.0);

        rightF.setPower(rightFP);
        leftB.setPower(leftBP);
        leftF.setPower(leftFP);
        rightB.setPower(rightBP);

        if (rightB.getPower() + leftF.getPower() != 0 ){
            //Круговое
            a = 1;
            x = 0;
        } else  if(rightB.getPower() < 0 && leftB.getPower() > 0 || rightB.getPower() > 0 && leftB.getPower() < 0 ){
            //Прямое
            x = 1;
            a = 0;
        }
        if(gamepad1.a || a == 1 ){
            telemetry.addLine("Круговое");
        } else if (gamepad1.x || x == 1) {
            telemetry.addLine("Прямое");
        }

        if (gamepad1.b && runtime2.milliseconds()<= 300){
            btimes ++;
            runtime2.reset();
        }else if (!gamepad1.b && btimes == 2 && runtime2.milliseconds()>= 300 ) {
            btimes = 0;
            runtime2.reset();
        }

        if(gamepad1.b){
            telemetry.addData("Напряга в rightB", rightB.getPower());
            telemetry.addData("Напряга в rightF", rightF.getPower());
            telemetry.addData("Напряга в leftF", leftF.getPower());
            telemetry.addData("Напряга в leftB", leftB.getPower());
            telemetry.addLine("\\");
            telemetry.addData("Скорость робота по X см/сек", velocityX * (CONSTS.DIST_BETWEEN_ENC_X/2));
            telemetry.addData("Скорость робота по Y см/сек", velocityY * (CONSTS.DIST_BETWEEN_ENC_X/2));
            telemetry.addData("Угловая скорость робота градус/сек",velocityAngle * (180/Math.PI));
            telemetry.addLine("\\");
            telemetry.addData("Скорость робота по X по джойстикам см/сек", targetVelAngleX * (CONSTS.DIST_BETWEEN_ENC_X/2));
            telemetry.addData("Скорость робота по Y по джойстикам см/сек", targetVelAngleY * (CONSTS.DIST_BETWEEN_ENC_X/2));
            telemetry.addData("Угловая скорость робота по джойстикам град/сек",targetVelAngle * (180/Math.PI));
            telemetry.addLine("\\");
            telemetry.addData("Левый джойстик X", gamepad1.left_stick_x);
            telemetry.addData("Левый джойстик Y", gamepad1.left_stick_y);
            telemetry.addData("Правый джойстик X",gamepad1.right_stick_x);
            telemetry.addLine("\\");
            telemetry.addData("Время", runtime.milliseconds());

        }else  {
            telemetry.addData("leftF тики", leftB.getCurrentPosition());
            telemetry.addData("Левый экодер тики", encL.getCurrentPosition());
            telemetry.addData("Правый энкодер тики", encR.getCurrentPosition());
            telemetry.addData("Серединный энкодер", encM.getCurrentPosition());
            telemetry.addLine("\\");

            telemetry.addData("Левый экодер см", encL.getCurrentPosition() / (CONSTS.TICK_PER_CM));
            telemetry.addData("Правый энкодер см", encR.getCurrentPosition() / (CONSTS.TICK_PER_CM));
            telemetry.addData("Серединный энкодер см", encM.getCurrentPosition() / (CONSTS.TICK_PER_CM));
            telemetry.addLine("\\");

            telemetry.addData("Левый экодер градус/сек", encL.getVelocity()/CONSTS.TICK_PER_DEGREES);
            telemetry.addData("Правый энкодер градус/сек", encR.getVelocity()/CONSTS.TICK_PER_DEGREES);
            telemetry.addData("Серединный градус/сек", encM.getVelocity()/CONSTS.TICK_PER_DEGREES);
            telemetry.addLine("\\");

            telemetry.addData("Угол робота", angle_robot);
            telemetry.addData("Угол робота глобал", currAngle);
            telemetry.addData("Угол синус через рад", Math.sin(Rad));
            telemetry.addData("Угол синус через град", Math.sin(angle_robot));
            telemetry.addData("Радиан", Rad);
            telemetry.addLine("\\");

            telemetry.addData("Rx", Rx);
            telemetry.addData("Ry", Ry);
            telemetry.addData("Gx", Gx);
            telemetry.addData("Gy", Gy);
            telemetry.addLine("\\");
            telemetry.addData("Время", runtime.milliseconds());
        }

        telemetry.update();

        runtime.reset();


        if (gamepad1.left_bumper && gamepad1.right_bumper){
            telemetry.addLine("Автоном мод ");
            double X = 0;
            double Y = 0;
            double RxA = 0;
            double RyA = 0;
            double forwardA = 0, sideA = 0, angleA = 0;
            double RadA = 0;
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
                    Rx = (((double) (encL.getCurrentPosition() + encR.getCurrentPosition()) /2) / CONSTS.TICK_PER_CM);
                } else  {
                    Rx = (((double) (encL.getCurrentPosition() - encR.getCurrentPosition()) /2) / CONSTS.TICK_PER_CM);
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
    }

}
