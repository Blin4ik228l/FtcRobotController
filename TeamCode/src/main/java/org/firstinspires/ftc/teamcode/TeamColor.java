package org.firstinspires.ftc.teamcode;

public class TeamColor {
    public final String red = "Red";
    public final String blue = "Blue";

    public TeamColor(String chosenColor){
        this.chosenColor = chosenColor;

        aprilTagIds = chosenColor.equals(red) ? aprilTagRedIds : aprilTagBlueIds ;
    }

    public int[] getTagIds(){

        return aprilTagIds;
    }
    public String chosenColor;

    public int[] aprilTagIds;

    public final int[] aprilTagBlueIds = {20};

    public final int[] aprilTagRedIds = {24};
}
