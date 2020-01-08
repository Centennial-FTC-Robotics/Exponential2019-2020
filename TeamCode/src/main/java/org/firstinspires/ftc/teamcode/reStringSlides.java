package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Exponential_Methods;


@TeleOp(name = "Slide Restring", group = "TeleOp")
public class reStringSlides extends Exponential_Methods {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();

        while(opModeIsActive()){
            slideDown.setPower(.25*gamepad1.left_stick_y);
            slideUp.setPower(.25*gamepad1.right_stick_y);
        }
    }
}
