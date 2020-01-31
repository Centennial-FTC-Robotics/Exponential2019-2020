package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Exponential_Methods;

@Autonomous(group = "Autonomous", name = "Red Corner")
public class RedCornerAuto extends Exponential_Methods {
    public void runOpMode() throws InterruptedException { // starts on second tile from the left
        super.runOpMode();
        waitForStart();
        //cornerAuto("red", false, false);
        cornerAutoSideways("red");
    }

}