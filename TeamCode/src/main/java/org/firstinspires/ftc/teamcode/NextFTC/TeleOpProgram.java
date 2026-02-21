package org.firstinspires.ftc.teamcode.NextFTC;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.NextFTC.Flywheel;
import org.firstinspires.ftc.teamcode.NextFTC.Intake;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;


@TeleOp(name = "NextFTC TeleOp Program Java")
public class TeleOpProgram extends NextFTCOpMode {

    private Follower follower;
    public static Pose startingPose;

    public TeleOpProgram() {
        addComponents(
                new SubsystemComponent(Flywheel.INSTANCE, Intake.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    public void intakeSequencing(){
        Intake.INSTANCE.On();
        Flywheel.INSTANCE.closeGate();
    }

    public void shooterSequencing(){
        Flywheel.INSTANCE.openGate();
        Intake.INSTANCE.On();
    }

    @Override public void onInit(){
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
    }

    public void shoot(){
        Flywheel.INSTANCE.On();
        telemetry.addData("works","ig");
        telemetry.update();
    }


    @Override
    public void onStartButtonPressed() {

        Flywheel.INSTANCE.closeGate();
        follower.startTeleopDrive();

        Gamepads.gamepad1().cross()
                .whenBecomesTrue(() -> shoot());
        Gamepads.gamepad1().circle()
                .whenBecomesTrue(() -> Flywheel.INSTANCE.Off());
        Gamepads.gamepad1().triangle()
                .whenBecomesTrue(() -> intakeSequencing())
                .whenBecomesFalse(() -> Intake.INSTANCE.Off());
        Gamepads.gamepad1().square()
                .whenBecomesTrue(() -> shooterSequencing());
    }
    @Override
    public void onUpdate(){
        follower.update();
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                true // Robot Centric
        );
    }

    public void onStop(){
        Flywheel.INSTANCE.closeGate();
    }
}

