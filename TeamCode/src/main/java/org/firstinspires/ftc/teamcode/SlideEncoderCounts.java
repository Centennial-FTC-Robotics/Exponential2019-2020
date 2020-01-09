package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name = "SlideEncoders", group = "TeleOp")
public class SlideEncoderCounts extends Exponential_Methods {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        while(opModeIsActive()){
            slideUp.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slideDown.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            telemetry.addData("Left Slide", slideDown.getCurrentPosition());
            telemetry.addData("Right Slide", slideUp.getCurrentPosition());
            telemetry.update();

        }




    }
}
