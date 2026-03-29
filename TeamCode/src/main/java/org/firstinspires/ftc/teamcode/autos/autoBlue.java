package org.firstinspires.ftc.teamcode.autos;


import static org.firstinspires.ftc.teamcode.nextFTCTeleOps.TeleOpTest.waitGate;
import static org.firstinspires.ftc.teamcode.nextFTCTeleOps.TeleOpTest.waitShoot;
import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;


@Autonomous(name = "Pedro Pathing Blue 12 Ball", group = "Autonomous")
@Configurable // Panels
public class autoBlue extends NextFTCOpMode {

    public autoBlue() {
        addComponents(
                new PedroComponent(Constants::createFollower)
        );
    }


    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    private int pathState; // Current autonomous path state (state machine)
    private ElapsedTime autoTimer = new ElapsedTime();


    @Override
    public void onInit() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        buildPaths();
        follower().setStartingPose(new Pose(18.998500749625187, 120.68725637181409, Math.toRadians(144)));
        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
        pathState = 0;


    }
    @Override
    public void onStartButtonPressed(){
        autonomousRoutine().run();
    }


    public Command autonomousRoutine() {
        return new SequentialGroup(
                new FollowPath(Path2),
                new FollowPath(Path3),
                new FollowPath(Path4),
                new FollowPath(Path5),
                new FollowPath(Path7),
                new FollowPath(Path8),
                new FollowPath(Path9),
                new FollowPath(Path10),
                new FollowPath(Path11),
                new FollowPath(Path12)
        );
    }


    @Override
    public void onUpdate() {
        // Log values to Panels and Driver Station
        panelsTelemetry.debug("X", follower().getPose().getX());
        panelsTelemetry.debug("Y", follower().getPose().getY());
        panelsTelemetry.debug("Heading", follower().getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }




    public PathChain Path2;
    public PathChain Path3;
    public PathChain Path4;
    public PathChain Path5;
    public PathChain Path7;
    public PathChain Path8;
    public PathChain Path9;
    public PathChain Path10;
    public PathChain Path11;
    public PathChain Path12;

    public void buildPaths() {
        Path2 = follower().pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(18.999, 120.687),
                                new Pose(52.073, 90.544),
                                new Pose(48.626, 85.565)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(121))

                .build();

        Path3 = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(48.626, 85.565),

                                new Pose(25.383, 84.017)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        Path4 = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(25.383, 84.017),

                                new Pose(48.965, 85.078)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))

                .build();

        Path5 = follower().pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(48.965, 85.078),
                                new Pose(51.552, 61.126),
                                new Pose(47.639, 55.752),
                                new Pose(24.765, 59.852)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(181))

                .build();

        Path7 = follower().pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(24.765, 59.852),
                                new Pose(22.309, 67.335),
                                new Pose(16.522, 68.687)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(270))

                .build();

        Path8 = follower().pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(16.522, 68.687)
                                ,
                                new Pose(27.965, 87.652),
                                new Pose(54.704, 80.078)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(131))

                .build();

        Path9 = follower().pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(54.704, 80.078),
                                new Pose(63.761, 31.309),
                                new Pose(24.643, 35.496)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(131), Math.toRadians(180))

                .build();

        Path9 = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(24.643, 35.496),

                                new Pose(51.548, 13.148)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(120))

                .build();

        Path10 = follower().pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(51.548, 13.148),
                                new Pose(7.087, 55.891),
                                new Pose(14.070, 29.717),
                                new Pose(11.322, 22.339)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(270))

                .build();

        Path11 = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(11.322, 22.339),

                                new Pose(50.730, 13.061)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(267), Math.toRadians(120))
                .setReversed()
                .build();

        Path11 = follower().pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(50.730, 13.061),
                                new Pose(9.300, 40.730),
                                new Pose(9.243, 13.443)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(270))

                .build();

        Path12 = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(9.243, 13.443),

                                new Pose(46.017, 10.643)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(101))

                .build();
    }

}