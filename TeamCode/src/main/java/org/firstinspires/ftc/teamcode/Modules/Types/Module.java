package org.firstinspires.ftc.teamcode.Modules.Types;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ConstansOrMagicNumbers.AnotherConsts;
import org.firstinspires.ftc.teamcode.ConstansOrMagicNumbers.Delays;
import org.firstinspires.ftc.teamcode.ConstansOrMagicNumbers.kPIDS;
import org.firstinspires.ftc.teamcode.ConstansOrMagicNumbers.ServoPositions;
import org.firstinspires.ftc.teamcode.Robot.GeneralInformation;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.VoltageSensorClass;

public class Module implements InterShow, InterResetTime, Delays, AnotherConsts, kPIDS, ServoPositions{
     public HardwareMap hardwareMap;
     public Telemetry telemetry;
     public VoltageSensorClass voltageSensor;
     public GeneralInformation generalInformation;
     public ElapsedTime innerRunTime;

     public Module(OpMode op){
          this.hardwareMap = op.hardwareMap;
          this.telemetry = op.telemetry;
          this.voltageSensor = new VoltageSensorClass(op);
          this.innerRunTime = new ElapsedTime();
     }

     @Override
     public void showData() {

     }

     @Override
     public void resetTimer() {
          this.innerRunTime.reset();
     }
}
