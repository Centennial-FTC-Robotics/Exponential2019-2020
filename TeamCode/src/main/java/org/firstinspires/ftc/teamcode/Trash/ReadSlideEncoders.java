package org.firstinspires.ftc.teamcode.Trash;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Exponential_Hardware_Initializations;

@TeleOp(name="Read Slides", group="TeleOp")
public class ReadSlideEncoders extends Exponential_Hardware_Initializations {
    public void runOpMode()throws InterruptedException{
        super.runOpMode();
        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("Up Slide", slideUp.getCurrentPosition());
            telemetry.addData("Down Slide", slideDown.getCurrentPosition());
            telemetry.update();
        }
    }
}
