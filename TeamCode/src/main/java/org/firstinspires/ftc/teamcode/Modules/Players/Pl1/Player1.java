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

        double forward = -1*playersGamepad.left_stick_y;
        double side = playersGamepad.left_stick_x;
        double turn = playersGamepad.right_stick_x;

        if (Math.abs(forward) < 0.1 && forward != 0){
            forward += 0.1 * Math.signum(forward);
        }
        if (Math.abs(side) < 0.1 && side != 0){
            side += 0.15 * Math.signum(side);
        }

        double forwardVoltage = Range.clip(forward * accelLinear , -max_speed, max_speed);
        double sideVoltage    = Range.clip(side * accelLinear ,  -max_speed, max_speed);
        double angleVoltage   = Range.clip(turn * accelAngle, -max_speed, max_speed);

        driveTrain.setPower(forwardVoltage, sideVoltage, angleVoltage);

        showData();
    }

    @Override
    public void showData() {
        driveTrain.odometry.getEncPos();
        driveTrain.odometry.getRobotPos();
    }


}

