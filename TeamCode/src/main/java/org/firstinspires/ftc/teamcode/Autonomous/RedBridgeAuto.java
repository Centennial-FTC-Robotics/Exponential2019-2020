package org.firstinspires.ftc.teamcode.Autonomous;

import org.firstinspires.ftc.teamcode.AutonomousPaths;

//@Autonomous(group = "Autonomous", name = "Red Bridge")
public class RedBridgeAuto extends AutonomousPaths {
    public void runOpMode() throws InterruptedException { // starts on third tile from the left
        super.runOpMode();
        waitForStart();
        bridgeAuto("red");

    }
}
