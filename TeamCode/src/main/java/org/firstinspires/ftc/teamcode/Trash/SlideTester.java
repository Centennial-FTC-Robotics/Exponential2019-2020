package org.firstinspires.ftc.teamcode.Trash;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Exponential_Methods;

@TeleOp(name = "SlideTester", group = "TeleOp")

public class SlideTester extends Exponential_Methods {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        waitForStart();
        while(opModeIsActive()) {
            telemetry.addData("stick", gamepad1.left_stick_y);
            telemetry.addData("up", slideUp.getCurrentPosition());
            telemetry.addData("down", slideDown.getCurrentPosition());
            telemetry.update();

            setSlidePower(0.3 * gamepad1.left_stick_y);


        }
    }
}
