package org.firstinspires.ftc.teamcode.Testers;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Exponential_Methods;

@TeleOp(name = "FrontRight")
public class MoveFrontRight extends Exponential_Methods {
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        frontRight.setPower(1);
        sleep(5000);
    }
}
