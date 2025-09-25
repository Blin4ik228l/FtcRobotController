package org.firstinspires.ftc.teamcode.Modules.Players.Pl1;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Modules.Players.Pl1.MecanumDriveTrain.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.Modules.Players.Player;

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
        double accelLinear = 1.3, accelAngle = 1.3;

        if(playersGamepad.left_trigger > 0.05 && playersGamepad.right_trigger < 0.05){//Ускорение робота
            accelLinear = 1.8;
            accelAngle = 1.8;
        }

        if(playersGamepad.right_trigger > 0.05 && playersGamepad.left_trigger < 0.05){//Замедление робота
            accelLinear = 0.25;
            accelAngle = 0.25;
        }

        double forward;
        double side;

        if(!joystickActivity.buttonA){
            forward = -1*playersGamepad.left_stick_y;
            side = playersGamepad.left_stick_x;
        }else{
            double[] globalVector = moveHeadless();

            forward = globalVector[0];
            side = globalVector[1];
        }

        double turn = playersGamepad.right_stick_x;

        if (Math.abs(forward) < 0.1 && forward != 0 && !joystickActivity.buttonA){
            forward += 0.1 * Math.signum(forward);
        }
        if (Math.abs(side) < 0.1 && side != 0 && !joystickActivity.buttonA){
            side += 0.15 * Math.signum(side);
        }

        double forwardVoltage = Range.clip(forward * accelLinear , -max_speed, max_speed);
        double sideVoltage    = Range.clip(side * accelLinear ,  -max_speed, max_speed);
        double angleVoltage   = Range.clip(turn * accelAngle, -max_speed, max_speed);

        driveTrain.setPower(forwardVoltage, sideVoltage, angleVoltage);

//        showData();
    }

    @Override
    public void showData() {
        driveTrain.odometry.getEncPos();
        driveTrain.odometry.getRobotPos();
    }

    public double[] moveHeadless(){
        double forward = -1*playersGamepad.left_stick_y;
        double side = playersGamepad.left_stick_x;

        double heading = driveTrain.odometry.getGlobalPosition().getHeading();

        double cosAngle = Math.cos((Math.PI/2) - heading);
        double sinAngle = Math.sin((Math.PI/2) - heading);// нам важен знак

        double globalSide = side * cosAngle - forward * sinAngle;
        double globalForward = forward * cosAngle + side * sinAngle;

        double[] globalVector = new double[2];

        globalVector[0] = globalForward;//Y
        globalVector[1] = globalSide;//X

        return globalVector;
    }

}

