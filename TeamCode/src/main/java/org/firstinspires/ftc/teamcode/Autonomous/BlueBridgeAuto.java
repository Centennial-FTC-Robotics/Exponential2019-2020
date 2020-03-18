package org.firstinspires.ftc.teamcode.Autonomous;

import org.firstinspires.ftc.teamcode.AutonomousPaths;

//@Autonomous(group = "Autonomous", name = "Blue Bridge")
public class BlueBridgeAuto extends AutonomousPaths {
    public void runOpMode() throws InterruptedException { // starts on third tile from the right
        super.runOpMode();
        waitForStart();
        bridgeAuto("blue");

    }
}
