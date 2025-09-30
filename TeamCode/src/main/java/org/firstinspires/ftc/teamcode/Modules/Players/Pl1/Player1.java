package org.firstinspires.ftc.teamcode.Modules.Players.Pl1;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.Modules.Players.Pl1.MecanumDriveTrain.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.Modules.Players.Player;

import java.util.concurrent.TimeUnit;

public class Player1 extends Player {
   public Player1(Gamepad gamepad, MecanumDriveTrain driveTrain, OpMode op){
       super(op.telemetry);

       playersGamepad = gamepad;
       this.driveTrain = driveTrain;
       joystickActivity = new JoystickActivity();
    }
   public MecanumDriveTrain driveTrain;
   public JoystickActivity joystickActivity;


    @Override
    public void play() {
        joystickActivity.checkActivity();

        double max_speed = 0.8;
        double acceleration = 1;

        double rightTrigga = playersGamepad.right_trigger;
        double leftTrigga = playersGamepad.left_trigger;

        if(leftTrigga > 0.05 && rightTrigga < 0.05){//Ускорение робота
            acceleration = 2;
        }

        if(rightTrigga > 0.05 && leftTrigga < 0.05){//Замедление робота
            acceleration = 0.5;
        }

        double cosA = playersGamepad.left_stick_x;
        double sinA = -1*playersGamepad.left_stick_y;
        double turn = playersGamepad.right_stick_x;

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

        driveTrain.setPower(forwardVoltage, sideVoltage, angleVoltage);

        showData();
    }

    @Override
    public void showData() {
        telemetry.addData("A",joystickActivity.buttonA);
        driveTrain.odometry.getEncPos();
        driveTrain.odometry.getRobotPos();
        driveTrain.gyro.getYaw();
    }

    public double[] moveHeadless(double cosA, double sinA){//FieldCentric
        double heading = -driveTrain.gyro.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);// "+" против часовой, "-" по часовой

        double cosB = Math.cos(heading);// где угол B это 90 +- угол робота относитеольно поля
        double sinB = Math.sin(heading);

        double globalX = cosA * cosB - sinA * sinB;
        double globalY = sinA * cosB + cosA * sinB;

        double[] globalVector = new double[2];

        globalVector[0] = globalX;
        globalVector[1] = globalY;

        return globalVector;
    }

}

