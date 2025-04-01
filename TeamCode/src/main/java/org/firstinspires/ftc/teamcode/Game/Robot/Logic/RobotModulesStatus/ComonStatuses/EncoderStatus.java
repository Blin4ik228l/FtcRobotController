package org.firstinspires.ftc.teamcode.Game.Robot.Logic.RobotModulesStatus.ComonStatuses;

public enum EncoderStatus {
    ZeroDelta,//Когда перемещение по энкорем нету
    SmallDelta,//Когда перемещение очень мало(при застревании)
    InMoving
}
