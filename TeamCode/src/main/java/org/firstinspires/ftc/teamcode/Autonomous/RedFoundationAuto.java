package org.firstinspires.ftc.teamcode.Autonomous;

import org.firstinspires.ftc.teamcode.AutonomousPaths;

//@Autonomous(group = "Autonomous", name = "Red Foundation")
public class RedFoundationAuto extends AutonomousPaths {
    public void runOpMode() throws InterruptedException { // starts on third tile from the right
        super.runOpMode();
        waitForStart();
        foundationAuto(0, "red");
    }
}
