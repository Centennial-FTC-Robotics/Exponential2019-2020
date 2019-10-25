package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(group = "Autonomous", name = "Tester")
public class Tester extends Exponential_Methods{
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        waitForStart();
        move(12,0,0.5);
        // turnAbsolute(90);
        while (opModeIsActive()) {
        }
    }
}
