package org.firstinspires.ftc.teamcode.NextFTC;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.dodgyLastMinute.Intake;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;



import dev.nextftc.ftc.NextFTCOpMode;


@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
@Configurable // Panels/*
public class BlueAUTOjade extends NextFTCOpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    private Intake intake = Intake.INSTANCE;
    private Flywheel flywheel = Flywheel.INSTANCE;
    private ElapsedTime autoTimer = new ElapsedTime();

    @Override
    public void onInit() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();


        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));


        flywheel.initialize();
        flywheel.closeGate();
        flywheel.setHoodAngle(50);


        paths = new Paths(follower); // Build paths


        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
        pathState = 0;


    }
    @Override
    public void onUpdate() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine


        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }




    public static class Paths {



        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;


        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(27.895, 130.612),


                                    new Pose(58.332, 96.901)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(133))


                    .build();


            Path2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(58.332, 96.901),
                                    new Pose(51.568, 80.692),
                                    new Pose(17.754, 83.182)
                            )
                    ).setTangentHeadingInterpolation()


                    .build();


            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(17.754, 83.182),


                                    new Pose(66.055, 80.252)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(128))
                    .setReversed()
                    .build();


            Path4 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(66.055, 80.252),
                                    new Pose(47.691, 82.022),
                                    new Pose(53.413, 68.030),
                                    new Pose(30.770, 52.065),
                                    new Pose(13.963, 62.833)
                            )
                    ).setTangentHeadingInterpolation()


                    .build();


            Path5 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(13.963, 62.833),
                                    new Pose(50.757, 35.861),
                                    new Pose(62.400, 22.330)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(120))
                    .setReversed()
                    .build();


            Path6 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(62.400, 22.330),
                                    new Pose(42.791, 38.352),
                                    new Pose(15.513, 35.417)
                            )
                    ).setTangentHeadingInterpolation()


                    .build();


            Path7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(15.513, 35.417),


                                    new Pose(51.261, 10.730)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(110))
                    .setReversed()
                    .build();
        }
    }




    public int autonomousPathUpdate() {


        /*switch (pathState) {

            case 0:
                follower.followPath(paths.Path1, true);
                pathState++;
                break;

            case 1:
                if (!follower.isBusy()) {
                    flywheel.On();
                    autoTimer.reset();
                    pathState++;
                }
                break;


            case 2:
                if (autoTimer.seconds() > 1.0) {
                    flywheel.openGate();
                    autoTimer.reset();
                    pathState++;
                }
                break;


            case 3:
                if (autoTimer.seconds() > 0.4) {
                    flywheel.closeGate();
                    flywheel.Off();
                    follower.followPath(paths.Path2, true);
                    intake.On();
                    pathState++;
                }
                break;




            case 4:
                if (!follower.isBusy()) {
                    intake.Off();
                    follower.followPath(paths.Path3, true);
                    pathState++;
                }
                break;




            case 5:
                if (!follower.isBusy()) {
                    flywheel.On();
                    autoTimer.reset();
                    pathState++;
                }
                break;


            case 6:
                if (autoTimer.seconds() > 1.0) {
                    flywheel.openGate();
                    autoTimer.reset();
                    pathState++;
                }
                break;


            case 7:
                if (autoTimer.seconds() > 0.4) {
                    flywheel.closeGate();
                    flywheel.Off();
                    follower.followPath(paths.Path4, true);
                    intake.On();
                    pathState++;
                }
                break;


            case 8:
                if (!follower.isBusy()) {
                    intake.Off();
                    follower.followPath(paths.Path5, true);
                    pathState++;
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    flywheel.On();
                    autoTimer.reset();
                    pathState++;
                }
                break;


            case 10:
                if (autoTimer.seconds() > 1.0) {
                    flywheel.openGate();
                    autoTimer.reset();
                    pathState++;
                }
                break;


            case 11:
                if (autoTimer.seconds() > 0.4) {
                    flywheel.closeGate();
                    flywheel.Off();
                    follower.followPath(paths.Path6, true);
                    intake.On();
                    pathState++;
                }
                break;


            case 12:
                if (!follower.isBusy()) {
                    intake.Off();
                    follower.followPath(paths.Path7, true);
                    pathState++;
                }
                break;


            // Final Shoot
            case 13:
                if (!follower.isBusy()) {
                    flywheel.On();
                    autoTimer.reset();
                    pathState++;
                }
                break;


            case 14:
                if (autoTimer.seconds() > 1.0) {
                    flywheel.openGate();
                    autoTimer.reset();
                    pathState++;
                }
                break;


            case 15:
                if (autoTimer.seconds() > 0.4) {
                    flywheel.closeGate();
                    flywheel.Off();
                    pathState++;
                }
                break;
        }


        return pathState;*/
        return 0;
    }
}