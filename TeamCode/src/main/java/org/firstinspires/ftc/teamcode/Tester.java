package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(group = "Autonomous", name = "Tester")
public class Tester extends Exponential_Methods{
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        waitForStart();

        while (opModeIsActive()) {

            move(12,12,0.5);
            wait(5000);
            turnAbsolute(90);
        }
    }
}
