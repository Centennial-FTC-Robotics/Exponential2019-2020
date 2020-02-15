package org.firstinspires.ftc.teamcode.Trash;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Exponential_Methods;

@TeleOp(name = "MoveTester", group = "TeleOp")

public class MoveTester extends Exponential_Methods {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        double sideways = 10;
        double forwards = 0;
        telemetry.addData("It got up here", "hello");
        telemetry.update();
        waitForStart();
        while(opModeIsActive()) {
            if(gamepad1.left_stick_x!=0||gamepad1.left_stick_y!=0) {
                sideways += .25*(int)(gamepad1.left_stick_x);
                forwards += .25*(int)(gamepad1.left_stick_y);
                sleep(200);
            }
            telemetry.addData("forwards", forwards);
            telemetry.addData("sideways", sideways);
            // telemetry.update();
            if(gamepad1.a){
                move(sideways, forwards, .5, .5);
            }

            if(gamepad2.x)
                setIntakeWheels(0.5);
            if(gamepad2.b)
                setIntakeWheels(0.5);
            if(gamepad2.y)
                setIntakeWheels(0);
            /*
            slideUp.setPower(gamepad2.left_stick_y * 0.4);
            slideDown.setPower(gamepad2.left_stick_y * 0.4);

            telemetry.addData("up", slideUp.getCurrentPosition());
            telemetry.addData("down", slideDown.getCurrentPosition());*/
            telemetry.update();
        }
    }
}
