package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import com.qualcomm.robotcore.util.ReadWriteFile;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.openftc.easyopencv.PipelineRecordingParameters;

import java.io.File;
import java.util.Base64;
import java.util.Timer;

@TeleOp(name="TelepopRed", group="Lagrange")
//@Disabled
public class TelepopRed extends LinearOpMode implements Inter{
    //Таймер
    Timer time = new Timer();
    //Железо
    private DcMotor leftRear, rightRear, rightFront, leftFront;
    private Servo s5;
    private DigitalChannel touch;

    //Переменные моторов

    private double zm1, zm2, zm3, zm4;
    private double last_moment_serv = 0.0, last_moment_switch = 0.0, last_moment_free = 0.0;
    private double moment_diff_serv, moment_diff_switch, moment_diff_free;
    private double a, turn;
    private ElapsedTime runtime = new ElapsedTime();
    private double lamp=0;
    private int height;
    int telescopePos;
    File telescopeFile = AppUtil.getInstance().getSettingsFile("telescopeFile.txt"); //Файл с позицией телескопа
    private int svob=0;
    //Гироскоп

    //Инициализируем железо
    public void initC() {
        //Инициализация
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        touch = hardwareMap.get(DigitalChannel.class, "touch");

        touch.setMode(DigitalChannel.Mode.INPUT);
    }

    @Override
    public void runOpMode() {

        class CalcThread implements Runnable {
            private Thread c;
            private boolean running;

            public void run() {
                telemetry.addLine("Calc thread running");
                telemetry.update();

                try {
                    leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                    while (!isStopRequested() & opModeIsActive()) {

                        //ТЕЛЕЖКА


                        //Коэффицент скорости робота
                        if(gamepad1.left_trigger<0.5){
                            a=0.7;
                        }
                        else if(gamepad1.left_trigger>0.5) {
                            a = 10;
                        }

                            //Поворот
                            turn = -gamepad1.right_stick_x;


                            //Мощность моторов тележки
                            zm1 = Range.clip((gamepad1.left_stick_x - gamepad1.left_stick_y - turn) * a, -1, 1);
                            if (zm1 > -0.05 && zm1 < 0.05) {
                                zm1 = 0;
                            }

                            zm2 = Range.clip((gamepad1.left_stick_x + gamepad1.left_stick_y - turn) * a, -1, 1);
                            if (zm2 > -0.05 && zm2 < 0.05) {
                                zm2 = 0;
                            }

                            zm3 = Range.clip((-gamepad1.left_stick_x + gamepad1.left_stick_y - turn) * a, -1, 1);
                            if (zm3 > -0.05 && zm3 < 0.05) {
                                zm3 = 0;
                            }

                            zm4 = Range.clip((-gamepad1.left_stick_x - gamepad1.left_stick_y - turn) * a, -1, 1);
                            if (zm4 > -0.05 && zm4 < 0.05) {
                                zm4 = 0;
                            }


                            //ТЕЛЕСКОП
                            moment_diff_serv = runtime.milliseconds() - last_moment_serv;
                            moment_diff_switch = runtime.milliseconds() - last_moment_switch;

                          
                            //Ручной захват
//                        if(touch.getState() == false){
//                            zs5 = CLOSE;
//                            lamp = -0.1;
//                            moment_diff_serv =205;
//                        }
//
//                     if(gamepad1.a==true && moment_diff_serv > 200) {
//                         if (zs5 == CLOSE) {
//                             zs5 = OPEN;
//                             lamp = 0;
//                             last_moment_serv = runtime.milliseconds();
//                         }else{
//                                 zs5 = CLOSE;
//                                 lamp = -0.1;
//                             last_moment_serv = runtime.milliseconds();
//                             }


                    }

                } catch (Exception e) {
                    telemetry.addLine("Calc thread interrupted");
                    telemetry.update();
                }
            }
            public void start_c() {
                if (c == null) {
                    c = new Thread(this, "Calc thread");
                    c.start();
                }
            }
        }

        //Инициализация
        initC();

        waitForStart();

        //Запуск подпроцессов
        CalcThread C1 = new CalcThread();
        C1.start_c();

        //ОСНОВНАЯ ПРОГРАММА

        while(opModeIsActive() & !isStopRequested()) {

            leftFront.setPower(zm1);//слева спереди
            rightFront.setPower(zm2);//справа спереди
            leftRear.setPower(zm3);//слева сзади
            rightRear.setPower(zm4);//справа сздади

//            En3.setPower(lamp);

            telemetry.addData("Состояние тригера", gamepad1.left_trigger);
            telemetry.addData("коэфицент скорости", a);
            telemetry.addData("Положение серво", height);
            telemetry.addData("Мотор1", -zm1);
            telemetry.addData("Мотор2", -zm2);
            telemetry.addData("Мотор3", zm3);
            telemetry.addData("Мотор4", zm4);
            telemetry.addData("Стик1 X", gamepad1.left_stick_x);
            telemetry.addData("Стик1 Y", gamepad1.left_stick_y);
            telemetry.addData("Стик2 X", gamepad2.right_stick_x);
            telemetry.addData("Стик2 Y", gamepad2.right_stick_y);
            telemetry.addData("Уровень", height);
            telemetry.addData("Ускорение", a);
            telemetry.update();

        }

        ReadWriteFile.writeFile(telescopeFile, Integer.toString(telescopePos));
    }
}