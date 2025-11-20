package org.firstinspires.ftc.teamcode.Robot.RobotParts.CollectorParts;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Modules.ExecutableModule;
import org.firstinspires.ftc.teamcode.Modules.MainModule;
import org.firstinspires.ftc.teamcode.Modules.Players.Pl2.AutomaticClass;
import org.opencv.core.Mat;

public class CollectorMotors extends MainModule {
    public CollectorMotors(OpMode op){
        super(op.telemetry);

        inTake = op.hardwareMap.get(DcMotor.class, "inTake");
        encMotorRight = op.hardwareMap.get(DcMotorEx.class, "flyWheelRight");
        encMotorLeft = op.hardwareMap.get(DcMotorEx.class, "flyWheelLeft");

        inTake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);// обновляем правый энкодер
        encMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);// обновляем левый энкодер

        encMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);//Запускаем
        encMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        inTake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        encMotorRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        encMotorLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        runTimeAll = new ElapsedTime();

        telemetry.addLine("Motors on collector inited");
    }
    private final DcMotor inTake;
    private final DcMotorEx encMotorRight;
    private final DcMotorEx encMotorLeft;
    public DcMotor getInTake() {
        return inTake;
    }
    public DcMotorEx getEncMotorLeft() {
        return encMotorLeft;
    }
    public DcMotorEx getEncMotorRight() {
        return encMotorRight;
    }
    public double curOverallVel, curLeftVel, curRightVel, inTakeCurPower;
    public double inTakePower;
    public double flyWheelVel;
    public ElapsedTime runTimeAll;
    public CollectorMotorsState collectorMotorsState;
    public enum CollectorMotorsState{
        Ready,
        Unready
    }
    private MotorsState motorsState;
    public MotorsState targetMotorState;
    public enum MotorsState{
        OnIntake,
        OffInTake,
        ReverseForWhile,
        OnFlyWheel,
        OffFlyWheel,
    }

    @Override
    public void update() {
        if (inTakePower != inTakeCurPower || !(Math.abs(flyWheelVel) <= Math.abs(curOverallVel)) && motorsState != targetMotorState) {
            collectorMotorsState = CollectorMotorsState.Unready;
            runTimeAll.reset();
        }else collectorMotorsState = CollectorMotorsState.Ready;
    }

    @Override
    public void execute() {
        if(motorsState != targetMotorState){
            motorsState = targetMotorState;
            switch (motorsState){
                case OnIntake:
                    inTakePower = -1;
                    flyWheelVel = 1;
                    break;
                case ReverseForWhile:
                    inTakePower = 1;
                    flyWheelVel = 0;
                    break;
                case OffInTake:
                    inTakePower = 0;
                    flyWheelVel = 0;
                    break;
                case OnFlyWheel:
                    flyWheelVel = 5;
                    break;
                case OffFlyWheel:
                    flyWheelVel = 0;
                    break;
            }
        }

        inTake.setPower(inTakePower);

        encMotorLeft.setPower(flyWheelVel);
        encMotorRight.setPower(-flyWheelVel);

//        encMotorLeft.setVelocity(flyWheelVel);
//        encMotorRight.setVelocity(-flyWheelVel);

        inTakeCurPower = inTake.getPower();

        curLeftVel = encMotorLeft.getVelocity(AngleUnit.RADIANS);
        curRightVel = encMotorRight.getVelocity(AngleUnit.RADIANS);

        curOverallVel = curLeftVel != 0 && curRightVel != 0 ? (curLeftVel - curRightVel) / 2.0 : curLeftVel - curRightVel;
    }


    @Override
    public void showData(){
        telemetry.addLine("===COLLECTOR MOTORS===");
        telemetry.addData("InTake Power", inTakeCurPower);
        telemetry.addData("FlyWheelSpeed overall","%.2f", curOverallVel);
        telemetry.addData("Left motor speed","%.2f", curLeftVel);
        telemetry.addData("Right motor speed","%.2f", curRightVel);
        telemetry.addData("Motors state", motorsState.toString());
        telemetry.addData("collectorMotorsState state", collectorMotorsState.toString());
        telemetry.addLine();
    }
}