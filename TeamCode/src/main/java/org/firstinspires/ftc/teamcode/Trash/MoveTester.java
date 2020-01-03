package org.firstinspires.ftc.teamcode.Trash;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Exponential_Methods;

@TeleOp(name = "MoveTester", group = "TeleOp")

public class MoveTester extends Exponential_Methods {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        double sideways = 0;
        double forwards = 0;
        telemetry.addData("It got up here", "hello");
        telemetry.update();
        waitForStart();
        while(opModeIsActive()) {
            if(gamepad1.left_stick_x!=0||gamepad1.left_stick_y!=0) {
                sideways += .1*(int)(gamepad1.left_stick_x);
                forwards += .1*(int)(gamepad1.left_stick_y);
                sleep(200);
            }
            telemetry.addData("forwards", forwards);
            telemetry.addData("sideways", sideways);
            telemetry.update();
            if(gamepad1.a){
                move(sideways, forwards, .5);
            }
        }
    }
}
