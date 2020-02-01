package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Exponential_Methods;

@Autonomous(group = "Autonomous", name = "Tile Right Forward")
public class TileRightForward extends Exponential_Methods {
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();
        tileSidewaysForwards("right");
    }
}
