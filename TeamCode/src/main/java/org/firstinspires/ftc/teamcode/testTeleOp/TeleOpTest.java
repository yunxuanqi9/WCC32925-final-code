package org.firstinspires.ftc.teamcode.testTeleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.dodgyLastMinute.Intake;
import org.firstinspires.ftc.teamcode.dodgyLastMinute.Shooter;
import org.firstinspires.ftc.teamcode.dodgyLastMinute.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
@Configurable
@TeleOp(name = "NextFTC TeleOp Test Java")
public class TeleOpTest extends NextFTCOpMode {

    public ElapsedTime gateTimer = new ElapsedTime();
    public static Pose startingPose = new Pose(0,12,Math.toRadians(90));
    public static double setHood = 0.79; // CHANGE IN PANELS
    public static double waitGate = 1;
    public static double waitShoot = 3;

    public boolean triggerRapidFire = false;
    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backLeftMotor;
    DcMotor backRightMotor;

    public TeleOpTest() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(
                        //Lift.INSTANCE,
                        //newFlywheel.INSTANCE,
                        Shooter.INSTANCE,
                        Intake.INSTANCE,
                        Turret.INSTANCE
                ),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    public Command shootArtifacts(){
        return new SequentialGroup(
          new SequentialGroup(
                  Shooter.INSTANCE.openGate,
                  new Delay(waitGate),
                  Shooter.INSTANCE.On

          ),
                new ParallelGroup(
                        Shooter.INSTANCE.closeGate,
                        Intake.INSTANCE.Off
                )
        );
    }

    @Override public void onWaitForStart(){
        PedroComponent.follower().setPose(startingPose);

    }

    @Override public void onInit(){
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        backRightMotor = hardwareMap.dcMotor.get("backRight");
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void onStartButtonPressed() {

        Shooter.INSTANCE.closeGate.schedule();
        //Shooter.INSTANCE.On.schedule(); // KSHITJ SAID SHOOTER DIDN'T WORK WITHOUT THIS?
        Shooter.INSTANCE.Off.schedule();
        Turret.INSTANCE.enableTracking.schedule();;

        Gamepads.gamepad1().circle()
                .toggleOnBecomesTrue()
                .whenBecomesTrue(Shooter.INSTANCE.On)
                .whenBecomesFalse(Shooter.INSTANCE.Off);
        Gamepads.gamepad1().rightBumper()
                .whenBecomesTrue(Intake.INSTANCE.On)
                .whenBecomesFalse(Intake.INSTANCE.Off);
        Gamepads.gamepad1().square()
                .whenBecomesTrue(new SequentialGroup(
                        Shooter.INSTANCE.openGate.thenWait(0.2),
                        Intake.INSTANCE.On.thenWait(2),
                        Intake.INSTANCE.Off,
                        Shooter.INSTANCE.closeGate
                ));

        Gamepads.gamepad1().options()
                .toggleOnBecomesTrue()
                .whenBecomesTrue(Turret.INSTANCE.enableTracking)
                .whenBecomesFalse(Turret.INSTANCE.disableTracking);
        Gamepads.gamepad1().dpadUp()
                .whenBecomesFalse(Shooter.INSTANCE.setSpeedHigh);
        Gamepads.gamepad1().dpadDown()
                .whenBecomesFalse(Shooter.INSTANCE.setSpeedLow);

    }
    @Override
    public void onUpdate(){
        telemetry.update();
        telemetry.addData("gatetimer",gateTimer.seconds());
        telemetry.addData("Shooter on",Shooter.INSTANCE.flywheelOn);

        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }

    public void onStop(){
        Shooter.INSTANCE.closeGate.schedule();
    }
}