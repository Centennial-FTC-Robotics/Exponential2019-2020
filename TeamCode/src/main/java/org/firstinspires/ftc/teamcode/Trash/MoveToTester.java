package org.firstinspires.ftc.teamcode.Trash;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.AutonomousPaths;
import org.firstinspires.ftc.teamcode.Position;

@Autonomous(group = "Autonomous", name = "moveTo tester")
public class MoveToTester extends AutonomousPaths {
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();
        initialHeading -= 90;
        currentPosition = new Position(0, 0);

        //top right -> top left -> bottom right -> origin

        double inches = 6;
        moveTo(inches, inches);
        moveTo(-inches, inches);
        moveTo(inches, -inches);
        moveTo(0, 0);
        sleep(1500);

        turnRelative(-90);
        moveTo(inches, inches);
        moveTo(-inches, inches);
        moveTo(inches, -inches);
        moveTo(0, 0);
        sleep(1500);

        turnRelative(-45);
        moveTo(inches, inches);
        moveTo(-inches, inches);
        moveTo(inches, -inches);
        moveTo(0, 0);
        sleep(1500);

        turnRelative(180);
        moveTo(inches, inches);
        moveTo(-inches, inches);
        moveTo(inches, -inches);
        moveTo(0, 0);
        /*moveTo(inches, inches);
        moveTo(0, inches);
        moveTo(-inches, -inches);
        moveTo(0, 0);*/

    }
}
