package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;

public class SlideExample extends OpMode {
    private DcMotorEx slideMotor;
    private ControlSystem controller;

    @Override
    public void init() {
        slideMotor = hardwareMap.get(DcMotorEx.class, "Turret");

        controller = ControlSystem.builder()
                .posPid(0.1, 0.0, 0.0)
                .elevatorFF(0.04)
                .build();

        controller.setGoal(new KineticState(0));
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            controller.setGoal(new KineticState(200));
        } else if (gamepad1.b) {
            controller.setGoal(new KineticState(0.0));
        } else if (gamepad1.x) {
            controller.setGoal(new KineticState(50));
        }

        slideMotor.setPower(controller.calculate(new KineticState(
                slideMotor.getCurrentPosition(),
                slideMotor.getVelocity()))
        );
    }
}