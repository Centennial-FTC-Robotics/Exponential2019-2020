package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Exponential_Methods;

@Autonomous(group = "Autonomous", name = "RedFoundationAuto")
public class RedFoundationAuto extends Exponential_Methods {
    public void runOpMode() throws InterruptedException { // starts on third tile from the right
        super.runOpMode();
        waitForStart();
        foundationAuto(0, "red");
    }
}
