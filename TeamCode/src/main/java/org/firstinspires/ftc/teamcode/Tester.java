package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(group = "Autonomous", name = "Tester")
public class Tester extends Exponential_Methods{
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        initializeIMU();

        waitForStart();
        //move(60, 0, .5); 
        move(0, 12 , 1.0/2000, 0,0,1,-1,.1);
        while (opModeIsActive()) {

        }
    }
}
