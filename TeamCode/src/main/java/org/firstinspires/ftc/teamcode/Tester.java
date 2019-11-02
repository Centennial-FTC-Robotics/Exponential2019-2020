package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(group = "Autonomous", name = "Tester")
public class Tester extends Exponential_Methods{
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        initializeIMU();

        waitForStart();
        turnAbsolute(90);
        telemetry.addData("turn 90 absolute", 0);
        move(12,0,0.3);
        telemetry.addData("moved forward", 0);
        turnRelative(45);
        telemetry.addData("turn 45 relative", 0);
        move(24,-12,0.3);
        telemetry.addData("strafe diagonal", 0);
        turnAbsolute(0);
        telemetry.addData("turn 0 absolute", 0);
        move(-12,0,0.3);
        telemetry.addData("moved back", 0);
    }
}
