package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(group = "Autonomous", name = "BlueCornerAuto")
public class BlueCornerAuto extends Exponential_Methods {

    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();

        while(opModeIsActive())
            cornerAuto("blue");

    }
}