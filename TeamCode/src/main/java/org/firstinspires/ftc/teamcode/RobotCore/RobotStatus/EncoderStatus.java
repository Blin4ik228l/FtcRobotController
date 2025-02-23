package org.firstinspires.ftc.teamcode.RobotCore.RobotStatus;

public enum EncoderStatus {
    ZeroDelta,//Когда перемещение по энкорем нету
    SmallDelta,//Когда перемещение очень мало(при застревании)
    InMoving
}
