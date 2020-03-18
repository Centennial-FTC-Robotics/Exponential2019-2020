package org.firstinspires.ftc.teamcode.Testers;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Exponential_Methods;

@TeleOp(name = "FrontLeft")
public class MoveFrontLeft extends Exponential_Methods {
    public void runOpMode() throws InterruptedException{
        super.runOpMode();
        frontLeft.setPower(1);
        sleep(5000);
    }
}
