package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.geometry.Pose;

public class autoConstants {
    public static Pose startingPose = new Pose();
    public static void writePose(Pose pose) {
        startingPose = pose;
    }
    public static double robotWidth = 16.221; //412 mm
    public static double robotCenterDistBack = 7.598; // 193mm
}
