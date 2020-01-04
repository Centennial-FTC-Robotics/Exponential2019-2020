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
                extendSlidesBy(4, 0.3);
            }
            if(gamepad2.a){
                extendSlidesBy(-4, 0.3); //change power later
            }


        }
    }
}
