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

            if(gamepad2.y){
                extendSlidesBy(6, 0.5);
            }
            if(gamepad2.a){
                extendSlidesBy(-6, 0.5);
            }
            if(gamepad2.b){
               toggleHook(true);
            }
            if(gamepad2.x) {
                toggleHook(false);
            }

            if(gamepad1.x){
                outwardsIntake();
            }
        }
    }
}