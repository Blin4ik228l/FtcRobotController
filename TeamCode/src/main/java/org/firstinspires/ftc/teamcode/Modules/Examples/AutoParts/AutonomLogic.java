package org.firstinspires.ftc.teamcode.Modules.Examples.AutoParts;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.ConstansOrMagicNumbers.ServoPositions;
import org.firstinspires.ftc.teamcode.Modules.Types.ExecutableModule;
import org.firstinspires.ftc.teamcode.Robot.RobotClass;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.Collector.Collector;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.Collector.CollectorParts.ColorSensorClass;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.Collector.CollectorParts.DigitalCellsClass;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DriveTrain.DrivetrainParts.Odometry.Parts.MathUtils.Position2D;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DriveTrain.DrivetrainParts.Odometry.Parts.MathUtils.Vector2;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DriveTrain.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.TaskAndArgs.Args;

public class AutonomLogic extends ExecutableModule {
    public AutonomLogic(RobotClass robotClass, OpMode op) {
        super(op.telemetry);
        this.robotClass = robotClass;

        driveTrain = robotClass.driveTrain;
        collector = robotClass.collector;
        digitalCellsClass = collector.digitalCellsClass;

        driveHandler = new DriveHandler(driveTrain, op);

        positionArtifactLogic = new PositionArtifactLogic(driveTrain.odometryClass, driveTrain.teamClass,  op);
        positionFireLogic = new PositionFireLogic(driveTrain, op);
        findTagsLogic = new FindTagsLogic(driveTrain, driveTrain.teamClass,op);
    }
    public RobotClass robotClass;
    public MecanumDrivetrain driveTrain;
    public Collector collector;
    public DigitalCellsClass digitalCellsClass;
    public DriveHandler driveHandler;

    public PositionArtifactLogic positionArtifactLogic;
    public PositionFireLogic positionFireLogic;
    public FindTagsLogic findTagsLogic;

    public RobotStates robotStates = RobotStates.Wait_For_Camera;
    public ProgramState programState = ProgramState.Load_state;
    public AutoState autoState = AutoState.Start_auto;

    public DriveLogic driveLogic = DriveLogic.Get_pos;
    public FireLogic fireLogic = FireLogic.Find_and_turn;

    public enum RobotStates{
        Wait_For_Camera,
        Find_tags,
        Idle,
        Rotate_Baraban,
        Check_Color,
        Go_to_pos,
        Near_pos,
        Prepare_to_fire,
        Fire
    }
    public enum DriveLogic{
        Get_pos,
        Power_motors,
        Done
    }
    public enum FireLogic{
        Find_and_turn,
        Check_readiness,
        Push_artifact,
        Check_artifact

    }
    public enum ProgramState{
        Load_state,
        Fire_state,
    }
    public enum AutoState {
        Start_auto,
        End_auto
    }

    double curAngle;
    double targetSpeed, curVelRad;
    /*
    * Пояснение к программе
    * Она построена по принципу State machine(машина состояний), что значит следущее:
    * Попадая в начальное состояние программа делает определённые действия свойственные для него, если определённые условия выполнены, то машина переходит в следующее состояние
    * Программа состоит из 2 основных состояний:
    * Load_state и Fire_state
    * В первом состоянии робота лежат действия по загрузки артифакта во внутр него
    * Вторая подпрограмма отвечает непосредственно за стрельбу артифактами
    * Затем программа повторяет цикл
    * */
    @Override
    public void execute() {
        double delayToBaraban = BARABAN_DELAY_FAR;
        double delayToPusher = PUSHERVER_DELAY;
        double delayToReverse = REVERSE_DELAY;

        switch (autoState){
            case Start_auto:
                //Если осталось мало времени и у нас есть загруженные артифакты => едем отстреливать что есть
                if(AUTO_TIME - robotClass.innerRunTime.seconds() < 4 && digitalCellsClass.getArtifactCount() != 0) {
                    autoState = AutoState.End_auto;
                    break;
                }
                switch (programState){
                    case Load_state:
                        switch (robotStates){
                            case Wait_For_Camera:
                                //TODO
                                robotStates = RobotStates.Check_Color;
                                driveTrain.motors.setPower(0,0,0);
//                                if(driveTrain.cameraClass.tagState == CameraClass.TagState.hasDetected && driveTrain.cameraClass.randomizeStatus == CameraClass.RandomizeStatus.Detected){
//                                    robotStates = RobotStates.Check_Color;
//                                    driveTrain.motors.setPower(0,0,0);
//                                    break;
//                                }
//                                if(driveTrain.cameraClass.generalLogic == CameraClass.GeneralLogic.Need_movement_ASAP){
//                                    robotStates = RobotStates.Find_tags;
//                                }
                                break;

                            case Find_tags:
                                findTagsLogic.execute();
                                robotStates = RobotStates.Wait_For_Camera;
                                break;

                            case Check_Color:
                                if(collector.colorSensorClass.colorState == ColorSensorClass.ColorSensorState.Artifact_Detected){
                                    digitalCellsClass.setColor(collector.colorSensorClass.getArtifactColor());
                                    digitalCellsClass.checkNumberOfArtifacts();

                                    //Это условие нужно, так как в начале в роботе уже предазагружены артифакты, чтобы мы не смогли удалить то чего нет
                                    if(positionArtifactLogic.getPosForSend() != null){
                                        positionArtifactLogic.deleteArtifact();
                                    }

                                    if(digitalCellsClass.getArtifactCount() == 3){
                                        robotStates = RobotStates.Idle;
                                    }else {
                                        robotStates = RobotStates.Rotate_Baraban;
                                    }
                                }
                                else {
                                    robotStates = RobotStates.Go_to_pos;
                                }
                                break;
                            case Rotate_Baraban:
                                double barabanPos = digitalCellsClass.getNextBarabanPos();

                                collector.servos.setBaraban(barabanPos);
                                if(isRotateEnded(delayToBaraban)){
                                    robotStates = RobotStates.Check_Color;
                                }

                                break;

                            case Go_to_pos:
                                switch (driveLogic){
                                    case Get_pos:
                                        positionArtifactLogic.update();

                                        if(positionArtifactLogic.logicStates == PositionArtifactLogic.LogicStates.Send_founded_pos) {

                                            driveHandler.setArgs(new Args.DriveArgs(
                                                    new Position2D(
                                                            Math.signum(positionArtifactLogic.getPosForSend().getX()) * (Math.abs(positionArtifactLogic.getPosForSend().getX()) - DRIVE_OFFSET),
                                                            positionArtifactLogic.getPosForSend().getY() * 1,
                                                            positionArtifactLogic.getPosForSend().getHeading() * 1
                                                    )
                                                    ,200)
                                            );
                                            driveLogic = DriveLogic.Power_motors;
                                        }


                                        break;
                                    case Power_motors:
                                        driveHandler.execute();

                                        if(driveHandler.isDone){
                                            driveHandler.isDone = false;
                                            driveLogic = DriveLogic.Done;
                                        }
                                        break;

                                    case Done:
                                        driveLogic = DriveLogic.Get_pos;
                                        robotStates = RobotStates.Near_pos;
                                        break;
                                }
                                break;

                            case Near_pos:
                                collector.motors.onIntake();

                                switch (driveLogic){
                                    case Get_pos:
                                        driveHandler.setArgs(new Args.DriveArgs(
                                                new Position2D(
                                                        positionArtifactLogic.getPosForSend().getX() * 1,
                                                        positionArtifactLogic.getPosForSend().getY() * 1,
                                                        positionArtifactLogic.getPosForSend().getHeading() * 1
                                                ),
                                                200)
                                        );
                                        driveLogic = DriveLogic.Power_motors;
                                        break;
                                    case Power_motors:
                                        driveHandler.execute();

                                        if(driveHandler.isDone){
                                            driveHandler.isDone = false;
                                            driveLogic = DriveLogic.Done;
                                        }

                                        break;

                                    case Done:
                                        driveTrain.motors.setPower(0,0,0);
                                        driveLogic = DriveLogic.Get_pos;

                                        robotStates = RobotStates.Check_Color;
                                        break;
                                }
                                break;

                            case Idle:
                                collector.motors.offIntake();

                                programState = ProgramState.Fire_state;
                                robotStates = RobotStates.Prepare_to_fire;
                                break;
                        }

                        break;
                    case Fire_state:
                        switch (robotStates){
                            case Prepare_to_fire:
                                collector.motors.preFireSpeedFlyWheel();

                                robotStates = RobotStates.Go_to_pos;
                                break;

                            case Go_to_pos:
                                switch (driveLogic){
                                    case Get_pos:
                                        positionFireLogic.update();
                                        if(positionFireLogic.logicStates == PositionFireLogic.LogicStates.Send_pos) {
                                            driveHandler.setArgs(new Args.DriveArgs(
                                                    new Position2D(
                                                            positionFireLogic.getSendedPos().getX() * 1,
                                                            positionFireLogic.getSendedPos().getY() * 1,
                                                            positionFireLogic.getSendedPos().getHeading() * 1
                                                    ),
                                                    200)
                                            );
                                            driveLogic = DriveLogic.Power_motors;
                                        };


                                        break;
                                    case Power_motors:
                                        driveHandler.execute();

                                        driveLogic = DriveLogic.Get_pos;

                                        if(driveHandler.isDone){
                                            driveHandler.isDone = false;
                                            driveLogic = DriveLogic.Done;
                                        }
                                        break;

                                    case Done:
                                        driveTrain.motors.setPower(0,0,0);
                                        driveLogic = DriveLogic.Get_pos;

                                        robotStates = RobotStates.Check_Color;
                                        break;
                                }
                                break;
                            case Fire:
                                double range = driveTrain.positionRobotController.getRange();

                                curAngle = getAngle3(range);//Находим начальный угол стрельбы
                                targetSpeed = getSpeed(range, curAngle);//Находим скорость на маховик

                                curVelRad = targetSpeed / MAX_EXPERIMENTAL_SPEED_IN_METERS * MAX_RAD_SPEED;
                                collector.servos.setAngle(findNeededPosAngle(curAngle));

                                collector.motors.setSpeedFlyWheel(curVelRad);
                                collector.servos.setPusherHor(ServoPositions.PUSHER_PREFIRE_POS);

                                Vector2 deltaVector = new Vector2(
                                        positionFireLogic.getSendedPos().getX() - driveTrain.odometryClass.getEncGlobalPosition2D().getX(),
                                        positionFireLogic.getSendedPos().getY() - driveTrain.odometryClass.getEncGlobalPosition2D().getY());

                                switch (fireLogic){
                                    case Find_and_turn:
                                        digitalCellsClass.checkNumberOfArtifacts();

                                        if (digitalCellsClass.getArtifactCount() == 0) {
                                            robotStates = RobotStates.Idle;
                                            break;
                                        }

//                                        // Определяем целевую ячейку (0, 1 или 2)
//                                        int targetCellIndex = 3 - digitalCellsClass.artifactCount; // 3→0, 2→1, 1→2
//                                        int targetColor = driveTrain.teamColorClass.getRandomizedArtifact()[targetCellIndex];
//
//                                        double targetPos = digitalCellsClass.findNeededArtifactPos(targetColor);


//                                        collector.servos.setBaraban(targetPos);
                                        if(isRotateEnded(delayToBaraban)){
                                            fireLogic = FireLogic.Check_readiness;
                                        }
                                        break;
                                    case Check_readiness:
                                        //Проверяем не сильно ли сдвинули другие роботы нашего
                                        if(deltaVector.length() > 10 && driveTrain.positionRobotController.getDeltaAngle() < Math.toRadians(5)){
                                            //TODO: Если не набирает скорость то?
//                                            if(Math.abs(collector.motors.cu - curVelRad) < 0.1){
//                                                fireLogic = FireLogic.Push_artifact;
//                                            }
                                        }else {
                                            robotStates = RobotStates.Go_to_pos;
                                        }
                                        break;
                                    case Push_artifact:
                                        collector.servos.setPusherHor(ServoPositions.PUSHERHOR_ENDING_POS);

                                        if (collector.servos.runTimePusherHor.seconds() > delayToPusher) {
                                            fireLogic = FireLogic.Check_artifact;
                                        }
                                        break;
                                    case Check_artifact:
                                        //Убираем толкатель так как он заслоняет датчик цвета
                                        collector.servos.setPusherHor(ServoPositions.PUSHER_PREFIRE_POS);

                                        if (collector.servos.runTimePusherHor.seconds() > delayToPusher) {
                                            if(collector.colorSensorClass.colorState == ColorSensorClass.ColorSensorState.No_Artifact_Detected){
                                                digitalCellsClass.deleteColorFromCell();
                                                fireLogic = FireLogic.Find_and_turn;
                                            }else {
                                                //Если шар не вылетелет => толкаем ещё раз
                                                fireLogic = FireLogic.Push_artifact;
                                            }
                                        }
                                        break;
                                }
                                break;
                            case Idle:
                                collector.motors.offFLyWheel();

                                collector.servos.setPusherHor(ServoPositions.PUSHER_START_POS);
                                if (collector.servos.runTimePusherHor.seconds() > delayToPusher) {
                                    programState = ProgramState.Load_state;
                                    robotStates = RobotStates.Check_Color;
                                }
                                break;
                        }
                        break;
                }
                break;
            case End_auto:
                switch (robotStates){
                    case Prepare_to_fire:
                        collector.motors.preFireSpeedFlyWheel();

                        robotStates = RobotStates.Go_to_pos;
                        break;
                    case Go_to_pos:
                        switch (driveLogic){
                            case Get_pos:
                                positionFireLogic.update();
                                driveHandler.setArgs(new Args.DriveArgs(
                                        new Position2D(
                                                positionFireLogic.getSendedPos().getX() * 1,
                                                positionFireLogic.getSendedPos().getY() * 1,
                                                positionFireLogic.getSendedPos().getHeading() * 1
                                        ),
                                        300)
                                );
                                driveLogic = DriveLogic.Power_motors;
                                break;
                            case Power_motors:
                                driveHandler.execute();

                                driveLogic = DriveLogic.Get_pos;

                                if(driveHandler.isDone){
                                    driveHandler.isDone = false;
                                    driveLogic = DriveLogic.Done;
                                }
                                break;

                            case Done:
                                driveTrain.motors.setPower(0,0,0);
                                driveLogic = DriveLogic.Get_pos;

                                robotStates = RobotStates.Check_Color;
                                break;
                        }
                        break;
                    case Fire:
                        double range = driveTrain.positionRobotController.getRange();

                        curAngle = getAngle3(range);//Находим начальный угол стрельбы
                        targetSpeed = getSpeed(range, curAngle);//Находим скорость на маховик

                        curVelRad = targetSpeed / MAX_EXPERIMENTAL_SPEED_IN_METERS * MAX_RAD_SPEED;
                        collector.servos.setAngle(findNeededPosAngle(curAngle));

                        collector.motors.setSpeedFlyWheel(curVelRad);
                        collector.servos.setPusherHor(ServoPositions.PUSHER_PREFIRE_POS);

                        Vector2 deltaVector = new Vector2(
                                positionFireLogic.getSendedPos().getX() - driveTrain.odometryClass.getEncGlobalPosition2D().getX(),
                                positionFireLogic.getSendedPos().getY() - driveTrain.odometryClass.getEncGlobalPosition2D().getY());

                        switch (fireLogic){
                            case Find_and_turn:
                                digitalCellsClass.checkNumberOfArtifacts();

                                if (digitalCellsClass.getArtifactCount() == 0) {
                                    robotStates = RobotStates.Idle;
                                    break;
                                }

                                // Определяем целевую ячейку (0, 1 или 2)
//                                int targetCellIndex = 3 - digitalCellsClass.artifactCount; // 3→0, 2→1, 1→2
//                                int targetColor = driveTrain.teamColorClass.getRandomizedArtifact()[targetCellIndex];

//                                double targetPos = digitalCellsClass.findNeededArtifactPos(targetColor);


//                                collector.servos.setBaraban(targetPos);
                                if(isRotateEnded(delayToBaraban)){
                                    fireLogic = FireLogic.Push_artifact;
                                }
                                break;
                                //Убираем это состояние так как здесь нам требуется скорость а не точность

//                            case Check_readiness:
//                                //Проверяем не сильно ли сдвинули другие роботы нашего
//                                if(deltaVector.length() > 10 && driveTrain.exOdometry.getDeltaAngle() < Math.toRadians(5)){
//                                    //TODO: Если не набирает скорость то?
//                                    if(Math.abs(collector.motors.curOverallVel - curVelRad) < 0.1){
//                                        fireLogic = FireLogic.Push_artifact;
//                                    }
//                                }else {
//                                    robotStates = RobotStates.Go_to_pos;
//                                }
//                                break;
                            case Push_artifact:
                                collector.servos.setPusherHor(ServoPositions.PUSHERHOR_ENDING_POS);

                                if (collector.servos.runTimePusherHor.seconds() > delayToPusher) {
                                    fireLogic = FireLogic.Check_artifact;
                                }
                                break;
                            case Check_artifact:
                                //Убираем толкатель так как он заслоняет датчик цвета
                                collector.servos.setPusherHor(ServoPositions.PUSHER_PREFIRE_POS);

                                if (collector.servos.runTimePusherHor.seconds() > delayToPusher) {
                                    if(collector.colorSensorClass.colorState == ColorSensorClass.ColorSensorState.No_Artifact_Detected){
                                        digitalCellsClass.deleteColorFromCell();
                                        fireLogic = FireLogic.Find_and_turn;
                                    }else {
                                        //Если шар не вылетелет => толкаем ещё раз
                                        fireLogic = FireLogic.Push_artifact;
                                    }
                                }
                                break;
                        }
                        break;
                    case Idle:
                        collector.motors.offFLyWheel();

                        collector.servos.setPusherHor(ServoPositions.PUSHER_START_POS);
                        if (collector.servos.runTimePusherHor.seconds() > delayToPusher) {
                            programState = ProgramState.Load_state;
                            robotStates = RobotStates.Check_Color;
                        }
                        break;
            }
                break;
        }
    }
    public double findNeededPosAngle(double curAngle){
        return ((90 - MAX_ANGLE) - (90 - Math.toDegrees(curAngle))) * (185 / 23) / 270;
    }
    double getAngle3(double range){
        return Math.atan(Math.tan(Math.toRadians(60)) + 2 * (100 - 30) / range);
    }
    double getSpeed(double range, double angle){
        return Math.sqrt(981 * range / ((Math.tan(Math.toRadians(60)) + Math.tan(angle)) * Math.pow(Math.cos(angle), 2))) / 100;
    }
    public boolean isRotateEnded(double delayToBaraban){
        return collector.servos.runTimeBaraban.seconds() > delayToBaraban;
    }

    @Override
    public void showData() {
        telemetry.addLine("===AUTO LOGIC===");
        telemetry.addData("AUTO state", autoState.toString());
        telemetry.addData("Program state", programState.toString());
        telemetry.addData("Robot state", robotStates.toString());
        telemetry.addLine();
    }
}

