package org.firstinspires.ftc.teamcode.Trash;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Exponential_Methods;

@TeleOp(name = "RotateTester", group = "TeleOp")
public class RotateTester extends Exponential_Methods {
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();
        double rotateAngle = 0;
        while(opModeIsActive()){
            if(gamepad1.left_stick_x!=0.0){
                rotateAngle+=(int)(10*gamepad1.left_stick_x);
                sleep(400);
            }
            telemetry.addData("Angle", rotateAngle);
            telemetry.update();
            if(gamepad1.a){
                turnRelative(rotateAngle);
            }
            if(gamepad1.b){
                turnAbsolute(rotateAngle);
            }
        }
    }
}

