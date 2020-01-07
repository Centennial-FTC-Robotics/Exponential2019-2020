package org.firstinspires.ftc.teamcode.Trash;

import org.firstinspires.ftc.teamcode.Exponential_Methods;

public class reStringSlides extends Exponential_Methods {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();

        while(opModeIsActive()){
            slideDown.setPower(gamepad1.left_stick_y);
            slideUp.setPower(gamepad1.right_stick_y);
        }
    }
}
