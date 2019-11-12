package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(group = "Autonomous", name = "Tester")
public class Tester extends Exponential_Methods{
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        initializeIMU();

        waitForStart();
        move(12,0,0.3);
        sleep(2000);
        move(0,-12,0.3);
        sleep(2000);
        move(12,12,0.3);
        sleep(2000);
        turnAbsolute(90);
        sleep(2000);
        move(30,0,0.3);
    }
}
