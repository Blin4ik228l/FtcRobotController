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

    double rho;
    double alpha;
    double delta;

    double radius;
    double center_x;
    double center_y;
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
        double speed = -playersGamepad.right_stick_y;
        double turn = playersGamepad.right_stick_x;

        currentAngle = driveTrain.exOdometry.encGlobalPosition.getHeading();
        currentTime = runtime.seconds();

        if (joystickActivity.buttonB){
            if(joystickActivity.tDpadUpPressed == 1){
//                constAngle = 0;
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
//                constAngle = constAngleStick;
            }

        }else {
            vyr = 0;
        }

        if(joystickActivity.buttonY){
            if(driveTrain.exOdometry.camera.getYaw() != 0){
                radius = driveTrain.exOdometry.camera.range;
                center_x = driveTrain.exOdometry.camera.ftcX;
                center_y = driveTrain.exOdometry.camera.ftcY;

                joystickActivity.buttonY = false;
                allowTurn = true;

            }
        }

        if (allowTurn) {
            double dx = center_x - driveTrain.exOdometry.encGlobalPosition.getX();
            double dy = center_y - driveTrain.exOdometry.encGlobalPosition.getY();

            rho = Math.sqrt(dx*dx + dy*dy);
            alpha = Math.atan2(dy, dx) - driveTrain.exOdometry.encGlobalPosition.getHeading();

//                alpha = Math.atan2(Math.sin(alpha), Math.cos(alpha));

        }

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

//        }

//        if(driveTrain.exOdometry.encGlobalPosition.getHeading() == Math.toRadians(-50) && allowTurn){
//            vyr = Math.toRadians(-50) - driveTrain.exOdometry.encGlobalPosition.getHeading();
//        }
//
//        if(driveTrain.exOdometry.encGlobalPosition.getHeading() == Math.toRadians(45) && allowTurn){
//            vyr = Math.toRadians(45) - driveTrain.exOdometry.encGlobalPosition.getHeading();
//        }

//        vyr = constAngle - driveTrain.exOdometry.encGlobalPosition.getHeading();//текущий угол - угол постояный, минус так как нужно провернутьсяв обратную сторону
//
        if(joystickActivity.buttonA){
            double[] globalVector = moveHeadless(cosA, sinA);

            cosA = globalVector[0];//X
            sinA = globalVector[1];//Y

            cosA *= 1.1;  // Counteract imperfect strafing
        }

        double v = cosA * (radius - rho);
        double omega = alpha;

        double denominator = Math.max(Math.abs(sinA) + Math.abs(cosA) + Math.abs(turn), 1);//Denominator is the largest motor power (absolute value) or 1

//        double forwardSpeed = (Math.abs(speed) * 300) * Math.signum(sinA);
//        double sideSpeed = (Math.abs(speed) * 300) * Math.signum(cosA);
//        double angleSpeed = turn * 6.28;
//
//        double forwardVoltage =   forwardSpeed / 300;
//        double sideVoltage    =   sideSpeed / 300;
//        double angleVoltage   =   angleSpeed / 6.28;

        double forwardVoltage =   sinA/(denominator * (1.0 / acceleration));
        double sideVoltage    =   cosA/(denominator * (1.0 / acceleration));
        double angleVoltage   =   turn/(denominator * (1.0 / acceleration));
        double vyrVoltage = 0;

        vyrVoltage = 0;


        driveTrain.setPower(forwardVoltage, sideVoltage, angleVoltage, vyrVoltage);


//        telemetry.addData("alpha", alpha);
//        telemetry.addData("constAngle", deltaConstAngle);
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
        driveTrain.exOdometry.showRobotNVel();
        driveTrain.exOdometry.showRobotVel();
        driveTrain.exOdometry.showRobotHeadingVel();
        driveTrain.exOdometry.showRobotAngleVel();
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

