package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(group = "Autonomous", name = "Tester")
public class Tester extends Exponential_Methods{
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        waitForStart();
        //move(60, 0, .5);
        move(60, 0, 1.0/20, 0,0,.5,-.5,.1);
        while (opModeIsActive()) {
            
        }
    }
}
