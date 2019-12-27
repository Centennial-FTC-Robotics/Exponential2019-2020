package org.firstinspires.ftc.teamcode;

public class RotateTester extends Exponential_Methods{
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();
        double rotateAngle = 0;
        while(opModeIsActive()){
            if(gamepad1.left_stick_x!=0){
                rotateAngle+=10*gamepad1.left_stick_x;
                wait(400);
            }
            if(gamepad1.a){
                turnRelative(rotateAngle);
            }
            if(gamepad1.b){
                turnRelative(rotateAngle);
            }
        }
    }
}

