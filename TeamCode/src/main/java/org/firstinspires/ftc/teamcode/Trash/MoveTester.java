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

        waitForStart();
        while(opModeIsActive()) {
            sideways+=gamepad1.left_stick_x;
            forwards+=gamepad1.left_stick_y;
            telemetry.addData("forwards", forwards);
            telemetry.addData("sideways", sideways);
            telemetry.update();
            sleep(400);
            if(gamepad1.a){
                move(forwards, sideways, .5);
            }
        }
    }
}
