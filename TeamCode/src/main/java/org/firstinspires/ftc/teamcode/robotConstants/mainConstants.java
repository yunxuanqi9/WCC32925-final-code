package org.firstinspires.ftc.teamcode.robotConstants;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

import org.opencv.core.Mat;

@Configurable
public class mainConstants {

    public static boolean redTeam = false;

    void isRedTeam(boolean isRed){
        redTeam = isRed;
    }

    public static double waitGateIntake = 0.2;
    public static double holdGate = 1;

    public static Pose autoEndPose;
    public static Pose gateIntake = new Pose(11.028, 62.042, Math.toRadians(145));

    public static double autoEndX = 0;
    public static double autoEndY = 0;
    public static double autoEndHeading = 0;




    public static double blueGateHeading = Math.toRadians(145);

    public static double gateHeading = blueGateHeading;
    public static double redGateHeading = Math.PI - blueGateHeading;


    public static void setEndPose(Pose pose){
        autoEndPose = pose;
    }

    public static Pose blueGoalPose = new Pose(5,133);
    public static Pose redGoalPose = blueGoalPose.mirror();
    public static Pose goalPose = blueGoalPose;


    public static void setAlliance(boolean isRed){
        redTeam = isRed;
        if(isRed){
            goalPose = redGoalPose;
            gateHeading = redGateHeading;
        }
        else{
            goalPose = blueGoalPose;
            gateHeading = blueGateHeading;
        }
    }


    public static double robotWidth = 16.93; //430 mm
    public static double robotCenterDistFront = 9.84; // 250mm

    public static double robotCenterDistBack = 7.598; // 193mm

    public static Pose bottomLeftCorner = new Pose(16.221/2,7.598,Math.toRadians(90));
    public static Pose bottomRightCorner = bottomLeftCorner.mirror();

}
