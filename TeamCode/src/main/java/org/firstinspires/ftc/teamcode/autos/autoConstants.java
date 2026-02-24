package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.geometry.Pose;

public class autoConstants {
    public static Pose startingPose = new Pose();
    public static void writePose(Pose pose) {
        startingPose = pose;
    }
}
