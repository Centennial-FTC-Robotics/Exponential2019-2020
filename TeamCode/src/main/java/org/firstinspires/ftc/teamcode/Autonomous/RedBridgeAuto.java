package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Exponential_Methods;

@Autonomous(group = "Autonomous", name = "RedBridgeAuto")
public class RedBridgeAuto extends Exponential_Methods {
    public void runOpMode() throws InterruptedException { // starts on third tile from the left
        super.runOpMode();
        waitForStart();
        while (opModeIsActive()) {
            bridgeAuto("red");
        }
    }
}
