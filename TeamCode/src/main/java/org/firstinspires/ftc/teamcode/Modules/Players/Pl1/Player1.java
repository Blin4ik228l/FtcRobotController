package org.firstinspires.ftc.teamcode.Modules.Players.Pl1;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

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

   public double constAngle = 0;
    public double constAngleDpad = 0;
    public double constAngleStick = 0;
   public double vyr = 0;
    double max_vel = 0;
   public boolean isRotateStarting = false;
   public boolean isRotateEnding = false;
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

        if (joystickActivity.buttonB){
            if(joystickActivity.tDpadUpPressed == 1){
                constAngle = 0;
                joystickActivity.tDpadUpPressed = 0;
            }

            if(turn != 0){
                isRotateStarting = true;
                isRotateEnding = false;
            }

            if(turn == 0 && isRotateStarting){
                isRotateEnding = true;
            }

            if(isRotateEnding){
                constAngleStick = driveTrain.exOdometry.gyro.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                isRotateEnding = false;
                isRotateStarting = false;
                constAngle = constAngleStick;
            }

//            if(joystickActivity.tDpadUpPressed == 1){
//                joystickActivity.tDpadRightPressed = 0;
//                joystickActivity.tDpadLeftPressed = 0;
//                joystickActivity.tDpadUpPressed = 0;
//            }
//
//            if(joystickActivity.tDpadLeftPressed == 0) {
//                switch (joystickActivity.tDpadRightPressed) {
//                    case 0:
//                        constAngleDpad = 0;
//                        break;
//                    case 1:
//                        constAngleDpad = Math.toRadians(-15);
//                        break;
//                    case 2:
//                        constAngleDpad = Math.toRadians(-30);
//                        break;
//                    case 3:
//                        constAngleDpad = Math.toRadians(-45);
//                        break;
//                    case 4:
//                        constAngleDpad = Math.toRadians(-60);
//                        break;
//                    case 5:
//                        constAngleDpad = Math.toRadians(-75);
//                        break;
//                    case 6:
//                        constAngleDpad = Math.toRadians(-90);
//                        break;
//                }
//            }
//            if(joystickActivity.tDpadRightPressed == 0) {
//                switch (joystickActivity.tDpadLeftPressed) {
//                    case 0:
//                        constAngleDpad = 0;
//                        break;
//                    case 1:
//                        constAngleDpad = Math.toRadians(15);
//                        break;
//                    case 2:
//                        constAngleDpad = Math.toRadians(30);
//                        break;
//                    case 3:
//                        constAngleDpad = Math.toRadians(45);
//                        break;
//                    case 4:
//                        constAngleDpad = Math.toRadians(60);
//                        break;
//                    case 5:
//                        constAngleDpad = Math.toRadians(75);
//                        break;
//                    case 6:
//                        constAngleDpad = Math.toRadians(90);
//                        break;
//                }
//            }
//
//            if(joystickActivity.tDpadRightPressed == 0 && joystickActivity.tDpadLeftPressed == 0 && joystickActivity.tDpadDownPressed == 1){
//                constAngle = constAngleStick;
//                joystickActivity.tDpadDownPressed = 0;
//            }else {
//                constAngle = constAngleDpad;
//            }

        }else {
            constAngle = driveTrain.exOdometry.gyro.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        }


        vyr = (driveTrain.exOdometry.gyro.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) - constAngle);//текущий угол - угол постояный, минус так как нужно провернутьсяв обратную сторону


        if(joystickActivity.buttonA){
            double[] globalVector = moveHeadless(cosA, sinA);

            cosA = globalVector[0];//X
            sinA = globalVector[1];//Y

            cosA *= 1.1;  // Counteract imperfect strafing
        }

        double denominator = Math.max(Math.abs(sinA) + Math.abs(cosA) + Math.abs(turn), 1);//Denominator is the largest motor power (absolute value) or 1

        double forwardVoltage = sinA/(denominator * (1.0 / acceleration));
        double sideVoltage    = cosA/(denominator * (1.0 / acceleration));
        double angleVoltage   = turn/(denominator * (1.0 / acceleration));
        double vyrVoltage = Math.signum(vyr) * 0.1;
//        double vyrVoltage = 0;
        if(Math.abs(vyr) < Math.toRadians(2)){
            vyrVoltage = 0;
        }


//        if((Math.abs(driveTrain.odometry.encL.getVelocity()) / 2000) * 0.096 * Math.PI > max_vel){
//            max_vel = (Math.abs(driveTrain.odometry.encL.getVelocity()) / 2000) * 0.096 * Math.PI;
//        }

        driveTrain.setPower(forwardVoltage, sideVoltage, angleVoltage, vyrVoltage);

        telemetry.addData("Max Vel", max_vel);
        showData();
    }

    @Override
    public void showData() {
//        driveTrain.exOdometry.showEncodersVel();
//        driveTrain.exOdometry.showRobotVel();
//        driveTrain.exOdometry.showEncodersAccel();
//        driveTrain.exOdometry.showRobotAccel();
        driveTrain.exOdometry.showEncPositions();
        driveTrain.exOdometry.showRobotPositionEnc();
        driveTrain.exOdometry.showRobotPositionGyro();

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

