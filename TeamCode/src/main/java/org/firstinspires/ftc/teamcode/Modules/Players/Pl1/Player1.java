package org.firstinspires.ftc.teamcode.Modules.Players.Pl1;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.teamcode.Modules.Players.Player;
import org.firstinspires.ftc.teamcode.Robot.RobotClass;

public class Player1 extends Player {
   public Player1(Gamepad gamepad, RobotClass.MecanumDrivetrain driveTrain, OpMode op){
       super(op.telemetry);

       playersGamepad = gamepad;
       this.driveTrain = driveTrain;
       joystickActivity = new JoystickActivity();
    }
   public RobotClass.MecanumDrivetrain driveTrain;
   public JoystickActivity joystickActivity;

   public double currentAngle = 0;
   public double lastAngle = 0;
   public double deltaAngle = 0;
   public double deltaConstAngle = 0;
   public double constAngle2 = 0;
   public boolean off1 = false;
    public boolean off2 = false;
   public boolean allowTurn = false;
    public double constAngleDpad = 0;
    public double constAngleStick = 0;
   public double vyr = 0;
    double max_vel = 0;
    public boolean flag = false;
   public boolean isRotateStarting = false;
   public boolean isRotateEnding = false;
double lastConstAngle = 0;
    public double x0 = 0;
    public double y0 = 0;
   public double rangeToTag = 0;
   public ElapsedTime runtime = new ElapsedTime();
   double currentTime = 0;
   double lastTime = 0;
   double deltaTime = 0;
    double circleLeng;
    @Override
    public void play() {
        joystickActivity.checkActivity();

        double max_speed = 0.8;
        double acceleration = 0.6;

        double rightTrigga = playersGamepad.right_trigger;
        double leftTrigga = playersGamepad.left_trigger;

        if(leftTrigga > 0.05 && rightTrigga < 0.05){//Ускорение робота
            acceleration = 2;
        }

        if(rightTrigga > 0.05 && leftTrigga < 0.05){//Замедление робота
            acceleration = 0.25;
        }

        double cosA = playersGamepad.left_stick_x;
        double sinA = -1*playersGamepad.left_stick_y;
        double turn = playersGamepad.right_stick_x;//для сохранения угла можно сделать её глобальной

        currentAngle = driveTrain.exOdometry.encGlobalPosition.getHeading();
        currentTime = runtime.seconds();

//        if (joystickActivity.buttonB){
//            if(joystickActivity.tDpadUpPressed == 1){
//                constAngle = 0;
//                joystickActivity.tDpadUpPressed = 0;
//            }
//
//            if(turn != 0){
//                isRotateStarting = true;
//                isRotateEnding = false;
//            }
//
//            if(turn == 0 && isRotateStarting){
//                isRotateEnding = true;
//            }
//
//            if(isRotateEnding){
//                constAngleStick = driveTrain.exOdometry.gyro.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
//                isRotateEnding = false;
//                isRotateStarting = false;
//                constAngle = constAngleStick;
//            }
//
//        }else {
            vyr = 0;
//        }

        if(joystickActivity.buttonY){
            if(driveTrain.exOdometry.camera.getYaw() != 0){
                rangeToTag = driveTrain.exOdometry.camera.range;
                driveTrain.exOdometry.encGlobalPosition.setY(driveTrain.exOdometry.camera.ftcY);
                driveTrain.exOdometry.encGlobalPosition.setX(driveTrain.exOdometry.camera.ftcX);
                driveTrain.exOdometry.encGlobalPosition.setHeading(driveTrain.exOdometry.camera.getYaw());
                driveTrain.exOdometry.gyroGlobalPosition.setHeading(driveTrain.exOdometry.camera.getYaw());
                joystickActivity.buttonY = false;
                allowTurn = true;

            }
        }

        if (allowTurn) {

            double velY = sinA * 200;
            double velX = cosA * 200;

            double y = driveTrain.exOdometry.encGlobalPosition.getY();
            double x = driveTrain.exOdometry.encGlobalPosition.getX();

            rangeToTag = Math.sqrt(Math.pow(y - rangeToTag * sinA, 2) + Math.pow(x - rangeToTag * cosA, 2));

            circleLeng = rangeToTag * 2 * Math.PI;

            x0 += driveTrain.exOdometry.selfMath.deltaX;
            y0 += driveTrain.exOdometry.selfMath.deltaY;

//            if(cosA != 0){
//                currentTime = runtime.seconds() - lastTime;
//                flag = false;
//            }


            if(cosA != 0){
                deltaConstAngle = (velX / circleLeng);
            }
             else{
                 deltaConstAngle = 0 ;
            }

            vyr = -driveTrain.exOdometry.selfMath.deltaHeadingEnc;

//            if(cosA == 0 && !flag){
//                lastTime = currentTime;
//                constAngle2 += deltaConstAngle;
//                flag = true;
//            }
//
//            if(deltaConstAngle != 0){
//                vyr = deltaConstAngle;
//            }
//            else{
//                constAngle2 -= driveTrain.exOdometry.encGlobalPosition.getHeading();
//                vyr = constAngle2;
//            }

        }

//        if(driveTrain.exOdometry.encGlobalPosition.getHeading() == Math.toRadians(-50) && allowTurn){
//            vyr = Math.toRadians(-50) - driveTrain.exOdometry.encGlobalPosition.getHeading();
//        }
//
//        if(driveTrain.exOdometry.encGlobalPosition.getHeading() == Math.toRadians(45) && allowTurn){
//            vyr = Math.toRadians(45) - driveTrain.exOdometry.encGlobalPosition.getHeading();
//        }

//        vyr = constAngle - driveTrain.exOdometry.encGlobalPosition.getHeading();//текущий угол - угол постояный, минус так как нужно провернутьсяв обратную сторону
//
//        if(joystickActivity.buttonA){
//            double[] globalVector = moveHeadless(cosA, sinA);
//
//            cosA = globalVector[0];//X
//            sinA = globalVector[1];//Y
//
//            cosA *= 1.1;  // Counteract imperfect strafing
//        }



        double denominator = Math.max(Math.abs(sinA) + Math.abs(cosA) + Math.abs(turn), 1);//Denominator is the largest motor power (absolute value) or 1

        double forwardVoltage =   sinA/(denominator * (1.0 / acceleration));
        double sideVoltage    =   cosA/(denominator * (1.0 / acceleration));
        double angleVoltage   =   turn/(denominator * (1.0 / acceleration));
        double vyrVoltage = Math.signum(vyr) * 0.1;

        vyrVoltage = 0;
//        double vyrVoltage = 0;
//        if(Math.abs(vyr) < Math.toRadians(2)){
//            vyrVoltage = 0;
//        }


//        if((Math.abs(driveTrain.odometry.encL.getVelocity()) / 2000) * 0.096 * Math.PI > max_vel){
//            max_vel = (Math.abs(driveTrain.odometry.encL.getVelocity()) / 2000) * 0.096 * Math.PI;
//        }
        deltaAngle = currentAngle - lastAngle;
        lastAngle = currentAngle;

        deltaTime = currentTime - lastTime;
        lastTime = currentTime;

        driveTrain.setPower(forwardVoltage, sideVoltage, angleVoltage, vyrVoltage);


        telemetry.addData("deltaHeadingEnc", driveTrain.exOdometry.selfMath.deltaHeadingEnc);
        telemetry.addData("constAngle", deltaConstAngle);
//        telemetry.addData("rangeToTag", rangeToTag);
//        telemetry.addData(" driveTrain.exOdometry.selfMath.deltaX",  driveTrain.exOdometry.selfMath.deltaX);
//        telemetry.addData("Vyr", vyr);
//        telemetry.addData("Max Vel", max_vel);
        showData();
    }

    @Override
    public void showData() {
//        driveTrain.exOdometry.showEncodersVel();
//        driveTrain.exOdometry.showRobotVel();
//        driveTrain.exOdometry.showEncodersAccel();
//        driveTrain.exOdometry.showRobotAccel();
//        telemetry.addData("DeltaYaw", driveTrain.exOdometry.camera.getDeltaFtcYaw());
        driveTrain.exOdometry.showEncPositions();
        driveTrain.exOdometry.showRobotPositionEnc();
//        driveTrain.exOdometry.showRobotPositionGyro();

//        telemetry.addData("A",joystickActivity.buttonA);
//        telemetry.addData("encLVel", driveTrain.odometry.encL.getVelocity());
//        telemetry.addData("encRVel", driveTrain.odometry.encR.getVelocity());
//        driveTrain.odometry.getEncPos();
//        driveTrain.odometry.getRobotPos();
//        driveTrain.gyro.getYaw();
    }

    public double[] moveHeadless(double cosA, double sinA){//FieldCentric
        double heading = -driveTrain.exOdometry.gyro.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);// "+" против часовой, "-" по часовой

        double cosB = Math.cos(heading);//  +- угол робота относитеольно поля
        double sinB = Math.sin(heading);

        double globalX = cosA * cosB - sinA * sinB;
        double globalY = sinA * cosB + cosA * sinB;

        double[] globalVector = new double[2];

        globalVector[0] = globalX;
        globalVector[1] = globalY;

        return globalVector;
    }

}

