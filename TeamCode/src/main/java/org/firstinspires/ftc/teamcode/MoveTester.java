package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "MoveTester", group = "TeleOp")

public class MoveTester extends Exponential_Methods {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        double sideways = 0;
        double forwards = 0;

        waitForStart();
        while(opModeIsActive()) {
            if(gamepad1.left_stick_x!=0||gamepad1.left_stick_y!=0) {
                sideways += .5*(int)(gamepad1.left_stick_x);
                forwards += .5*(int)(gamepad1.left_stick_y);
                sleep(400);
            }
            telemetry.addData("forwards", forwards);
            telemetry.addData("sideways", sideways);
            telemetry.update();
            if(gamepad1.a){
                move(forwards, sideways, .5);
            }
        }
    }
}
