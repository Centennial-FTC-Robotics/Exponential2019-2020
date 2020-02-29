package org.firstinspires.ftc.teamcode.Trash;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Exponential_Methods;

@TeleOp(name = "MoveTester", group = "TeleOp")

public class MoveTester extends Exponential_Methods {

    double p = -0.000055;
    double i = -0.00001;
    double d = 0.000011;

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        double sideways = 36;
        double forwards = 0;
        telemetry.addData("It got up here", "hello");
        telemetry.update();
        waitForStart();
        while(opModeIsActive()) {
            if(gamepad1.left_stick_x!=0||gamepad1.left_stick_y!=0||gamepad1.right_stick_x!=0||gamepad1.right_stick_y!=0||gamepad2.right_stick_x!=0) {
                sideways += 1*(int)(gamepad1.left_stick_x);
                forwards -= 1*(int)(gamepad1.left_stick_y);
                p+=0.000001*(int)(gamepad1.right_stick_x);
                i-=0.000001*(int)(gamepad1.right_stick_y);
                d+=0.000001*(int)(gamepad2.right_stick_x);
                sleep(200);

            }
            telemetry.addData("forwards", forwards);
            telemetry.addData("sideways", sideways);
            telemetry.addData("p value", p);
            telemetry.addData("i value", i);
            telemetry.addData("d value", d);
            telemetry.update();
            if(gamepad1.a){
                move(sideways, forwards, p, i, d, 1);
            }
            telemetry.update();
        }
    }
}
