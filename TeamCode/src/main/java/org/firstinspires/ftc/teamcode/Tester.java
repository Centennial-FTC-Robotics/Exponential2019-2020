package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(group = "Autonomous", name = "Tester")
public class Tester extends Exponential_Methods{
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        initializeIMU();

        waitForStart();
        turnRelative(90);
        move(24,0,0.5);
        move(0,24,0.5);
        move(-24,0,0.5);
        move(0,-24,0.5);
        sleep(1000);
        turnAbsolute(0);
        sleep(1000);
        move(36,0,1);
    }
}
