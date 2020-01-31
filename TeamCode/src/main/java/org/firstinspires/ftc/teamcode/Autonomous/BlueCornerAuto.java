package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Exponential_Methods;

@Autonomous(group = "Autonomous", name = "Blue Corner")
public class BlueCornerAuto extends Exponential_Methods {
    public void runOpMode() throws InterruptedException { // starts on second tile from the right
        super.runOpMode();
        waitForStart();
        //cornerAuto("blue", true, false);
        cornerAutoSideways("blue");
    }

}