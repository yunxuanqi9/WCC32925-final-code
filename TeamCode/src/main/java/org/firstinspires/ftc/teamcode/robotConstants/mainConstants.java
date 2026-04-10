package org.firstinspires.ftc.teamcode.robotConstants;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

@Configurable
public class mainConstants {

    public static boolean redTeam = false;

    void isRedTeam(boolean isRed){
        redTeam = isRed;
    }


    public static Pose autoEndPose;

    public static double autoEndX;
    public static double autoEndY;

    public static void setEndPose(Pose pose){
        autoEndPose = pose;
    }

    public static Pose blueGoalPose = new Pose(2,144);
    public static Pose redGoalPose = blueGoalPose.mirror();
    public static Pose goalPose = blueGoalPose;




    public static Pose startingPose = new Pose();
    public static void writePose(Pose pose) {
        startingPose = pose;
    }
    public static double robotWidth = 16.221; //412 mm
    public static double robotCenterDistBack = 7.598; // 193mm

    public static Pose bottomLeftCorner = new Pose(16.221/2,7.598,Math.toRadians(90));
    public static Pose bottomRightCorner = bottomLeftCorner.mirror();

}
