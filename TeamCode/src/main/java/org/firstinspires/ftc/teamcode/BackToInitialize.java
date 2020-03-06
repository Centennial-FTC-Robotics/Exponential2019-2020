package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Exponential_Methods;

@TeleOp(name = "Back to Initialize", group = "TeleOp")

public class BackToInitialize extends Exponential_Methods {
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        while(opModeIsActive()){
            if(gamepad1.x){
                extendSlidesTo(600, 0.4);
                sleep(2000);
                intakeServoRight.setPosition(0.3);
                intakeServoLeft.setPosition(0);
            }
            if(gamepad1.b){
                yuhwanSlidesDown();
            }

        }
    }
}