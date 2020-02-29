package org.firstinspires.ftc.teamcode.Autonomous;

import org.firstinspires.ftc.teamcode.Exponential_Methods;
import org.firstinspires.ftc.teamcode.Position;

public class AutonomousPaths extends Exponential_Methods {

    public void twoStoneAutoMoveTo(String color, int stonePos) {
        initialHeading -= 270;

        int factor;
        if (color.equals("red"))
            factor = 1;
        else
            factor = -1;


        //start distance away from wall (set later)
        double startX = 3 * TILE_LENGTH * factor;
        double startY = /*TILE_LENGTH * 2 - ROBOT_LENGTH*/ TILE_LENGTH /* + something */;

        currentX = startX;
        currentY = startY;

        int inchesBlocks = stonePos * 8;

        setTargetAngle(270);

        moveRelative(factor * (TILE_LENGTH + MIDDLE_OF_TILE - 6), 0);
        yuhwanSlidesDown();
        moveRelative(factor * 6, 0);
        double intakeWidthOffset = 2;

        if (stonePos == 2) {
            setTargetPosition(targetX, -TILE_LENGTH + MIDDLE_OF_TILE);

            turnAbsolute(270 - factor * 45);
            //turnAbsolute(225);

            double halfOfDiagonal = Math.sqrt(2 * Math.pow(TILE_LENGTH, 2)) / 2;

            double diagonalToStone = halfOfDiagonal - ROBOT_LENGTH / 2 - intakeWidthOffset;

            move(0, diagonalToStone);
            yuhwanIntakeStone();
            move(0, -diagonalToStone);

            turnAbsolute(270);
            //turnRelative(factor * 45);
        } else {
            setTargetPosition(targetX, -2 * TILE_LENGTH + inchesBlocks + intakeWidthOffset);
            moveRelative(factor * -TILE_LENGTH / 2, 0);

            yuhwanIntakeStone();

            moveRelative(factor * TILE_LENGTH / 2, 0);
        }
        setTargetPosition(targetX, FOUNDATION_POSITION_MOVETO);

        //releasing stone
        extendSlidesBy(6, .5);

        turnAbsolute(90 + factor * 90);
        //turnAbsolute(180);

        moveRelative(-8 * factor, 0);
        releaseStone();

        //preparing for foundation
        moveRelative(8 * factor, 0);

        turnAbsolute(90 - factor * 90);
        //turnAbsolute(0);

        extendSlidesBy(-6, .5);
        outwardsIntake();

        moveRelative(-11 * factor, 0);
        sleep(500);
        toggleHook(true);
        sleep(250);

        //rotating foundation

        setTargetPosition(factor * (3 * TILE_LENGTH - MIDDLE_OF_TILE), targetY);

        turnAbsolute(270);

        toggleHook(true);
        //centering with second tile
        setTargetPosition(factor * (2 * TILE_LENGTH - MIDDLE_OF_TILE), targetY);
        //moving to intake second block
        setTargetPosition(targetX, -3 * TILE_LENGTH + inchesBlocks + intakeWidthOffset);

        moveRelative(factor * -TILE_LENGTH / 2, 0);
        yuhwanIntakeStone();
        moveRelative(factor * TILE_LENGTH / 2, 0);
        //moving to 4th tile to rotate and extend slides
        setTargetPosition(targetX, TILE_LENGTH);
        extendSlidesBy(6, .5);

        turnAbsolute(90);
        //turnRelative(180);

        //placing the stone
        setTargetPosition(targetX, TILE_LENGTH * 3 - ROBOT_LENGTH - FOUNDATION_WIDTH);
        releaseStone();
        //moving back to lower slides
        moveRelative(0, -TILE_LENGTH);
        extendSlidesBy(-6, .5);
        outwardsIntake();
        //park
        setTargetPosition(targetX, -ROBOT_LENGTH / 2);

    }
    public void twoStoneAuto(String color, int stonePos) { //starts facing the bridge
        //initialHeading -= 270; //robot starts off facing 270

        int factor;
        if (color.equals("red"))
            factor = 1;
        else
            factor = -1;


        //start distance away from wall (set later)
        double startX = /*TILE_LENGTH * 2 - ROBOT_LENGTH *//*- MIDDLE_OF_TILE*/TILE_LENGTH;
        double startY = 0;

        currentX = startX;
        currentY = startY;

        int inchesBlocks = stonePos * 8;

        move(factor * (TILE_LENGTH + MIDDLE_OF_TILE - startY - 6), 0);
        yuhwanSlidesDown();
        //move(0, factor * 6);

        double intakeWidthOffset = 2;


        if (stonePos == 2) {
            move(0, startX - (TILE_LENGTH * 2 + MIDDLE_OF_TILE));
            //move the front of the robot to the almost corner of the tile
            turnAbsolute(factor * -45);
            //turnRelative(factor * -45);

            double halfOfDiagonal = Math.sqrt(2 * Math.pow(TILE_LENGTH, 2)) / 2;
            double diagonalToStone = halfOfDiagonal - ROBOT_LENGTH / 2 - intakeWidthOffset;
            move(0, diagonalToStone);
            yuhwanIntakeStone();
            move(0, -diagonalToStone);
            turnAbsolute(0);
            //turnRelative(factor * 45);

            //moving to foundation
            move(0, (2 * TILE_LENGTH + MIDDLE_OF_TILE) - FOUNDATION_POSITION);
        } else {
            //move(0, startX - (TILE_LENGTH + MIDDLE_OF_TILE));
            //move(0, MIDDLE_OF_TILE - 10 + inchesBlocks);
            move(0, startX - (TILE_LENGTH + intakeWidthOffset + BLOCK_LENGTH + inchesBlocks));
            move(factor * (TILE_LENGTH / 2 + 6), 0);

            yuhwanIntakeStone();
            move(factor * (-TILE_LENGTH / 2 - 1 - 1), 0);

            move(0, (TILE_LENGTH + intakeWidthOffset + BLOCK_LENGTH + inchesBlocks + 4) - FOUNDATION_POSITION);
        }
        //currently at middle of foundation

        //releasing stone
        extendSlidesBy(6, .5);
        turnAbsolute(factor * -90);
        //turnRelative(factor * -90);

        move(0, 8);
        releaseStone();

        //preparing for foundation
        move(0, -8);
        turnAbsolute(factor * 90);
        //turnRelative(180);

        extendSlidesBy(-6, .5);
        outwardsIntake();

        move(0, -8, .3);
        //sleep(500);
        toggleHook(true);
        sleep(150);

        move(0, (8 + TILE_LENGTH + MIDDLE_OF_TILE + 2) - MIDDLE_OF_TILE);

        //rotating foundation
        //double inchesToPlaceFoundation =
        turnAbsolute(0);
        //turnRelative(factor * -90);

        toggleHook(false);
        //centering with second tile
        move(factor * (TILE_LENGTH + MIDDLE_OF_TILE - MIDDLE_OF_TILE/*TODO*/ - 6), 0);
        //moving to intake second block
        move(0, 6 * TILE_LENGTH - FOUNDATION_WIDTH - ROBOT_LENGTH - intakeWidthOffset - BLOCK_LENGTH - inchesBlocks - 2/* - 2*/);



        move(factor * TILE_LENGTH / 2, 0);
        yuhwanIntakeStone();
        move(factor * -TILE_LENGTH / 2, 0);
        //moving to 4th tile to rotate and extend slides
        move (0, -(4 * TILE_LENGTH - 10 - inchesBlocks - 5));
        extendSlidesBy(6, .5);
        turnAbsolute(180);
        //turnRelative(180);

        //placing the stone
        move(0, 2 * TILE_LENGTH - ROBOT_LENGTH - FOUNDATION_WIDTH + 2);
        releaseStone();
        //moving back to lower slides
        move(0, -TILE_LENGTH);
        extendSlidesBy(-6, .5);
        outwardsIntake();
        //park
        move(0, (TILE_LENGTH * 3 - ROBOT_LENGTH / 2) - (5 * TILE_LENGTH - FOUNDATION_WIDTH - ROBOT_LENGTH));

    }
    public void cornerAutoSideways(String color, int stonePos) { //starts facing the bridge
        int factor;
        if (color.equals("red"))
            factor = 1;
        else
            factor = -1;

        //double MAX_POWER = 0.6;

        //start distance away from wall (set later)
        double startX = TILE_LENGTH * 2 - ROBOT_LENGTH /*- MIDDLE_OF_TILE*/;
        double startY = 0;

        //position of skystone (start later)

        //int numBlocks = stonePos;
        //int numBlocks = left ? 0 : center ? 1 : right ? 2: -1;
        int inchesBlocks = stonePos * 8;
        double intakeOffset = TILE_LENGTH - ROBOT_LENGTH/* + 1*/; //TODO: change the number later, inches to get the robot close enough to the block

        //outwardsIntake();

        double positionForSkystone = inchesBlocks + intakeOffset;
        move(factor * -startY, -(startX -positionForSkystone)); //moves to the correct horizontal position


        double stopMiddleAmount = 18;

        double randomAlign = 2;
        move(factor * -(TILE_LENGTH * 2 - ROBOT_LENGTH / 2 - stopMiddleAmount - randomAlign), 0); //moves sideways to get in intaking position
        //bringSlidesDown();

        yuhwanSlidesDown();

        move(factor * -stopMiddleAmount, 0);
        //intaking stone
        clampStone();
        setIntakeWheels(.7);
        sleep(1500);
        stopIntakeWheels();

        //move to center of second tile
        move(factor * (ROBOT_LENGTH / 2 + MIDDLE_OF_TILE + randomAlign + 2), 0);

        double foundationPosition = (TILE_LENGTH * 6 - ROBOT_LENGTH - 4 - FOUNDATION_LENGTH / 2);
        //move to foundation
        //moveSetISetP(0, -positionForSkystone + foundationPosition /*TILE_LENGTH * 4.5*/, .5, 1, .0003, 1.0/1000);
        move(0, -positionForSkystone + foundationPosition);

        //releasing stone
        extendSlidesBy(6, .5);
        turnRelative(factor * 90);
        move(0, 8);
        releaseStone();
        //preparing for foundation
        move(0, -8);
        turnRelative(180);
        extendSlidesBy(-6, .5);
        outwardsIntake();

        move(0, -11, .3);
        sleep(500);
        toggleHook(true);
        sleep(1000);
        //right now vert position: 9 inches + middle of second tile
        double inchesToPlaceFoundation = 0;
        move(0, 11  + (((20))) + inchesToPlaceFoundation);
        turnRelative(factor * -90);
        toggleHook(false);
        move(factor * 9, 0);
        move(factor * inchesToPlaceFoundation, 0);

        move(0, foundationPosition - (3 * TILE_LENGTH - ROBOT_LENGTH / 2) - 15);


    }
    public void cornerAuto(String color, boolean second, boolean secondTilePath) { // starts on second tile from the side
        int factor;
        if (color.equals("red"))
            factor = 1;
        else
            factor = -1;

        double MAX_POWER = 0.6;

        bringSlidesDown();
        //coordinates are for red side, they represent the location of the bottom left point of the robot from our POV
        // no matter what direction the robot is facing. done to hopefully reduce confusion cause fuck trying to
        //figure out what was going on

        // ROBOT DISTANCE AWAY FROM BLOCK
        // this variable determines how far away from the block we want the robot when using grabSkystone
        // using this variable, calculate the distance the robot must travel to get the middle of stone exactly
        // in robot's field of sight
        double observingDistance = 13;
        // these two variables are separate for code clarity
        // idk what I would name a variable that represents both of these values
        double observingDistanceX = observingDistance / Math.sqrt(2);
        double observingDistanceY = observingDistance / Math.sqrt(2);

        double secondTile = TILE_LENGTH;
        double offsetForFoundation = 3; // TO BE CHANGED IF NEED BE
        // (1 tile, 0)

        double forwardToGetStone = 2 * TILE_LENGTH - ROBOT_LENGTH - observingDistanceY;
        moveAddTolerance(-factor * (TILE_LENGTH - observingDistanceX), 0, MAX_POWER, .2); //move to corner //(observing distance x, 0)
        moveAddTolerance(0, forwardToGetStone, MAX_POWER, .1); //move forward towards stones //(obs. dist. x, forwardToGetStone)
        int inchesMoved = grabSkystone(color); //(obs.dist.x + x, forwardsToGetStone)
        /*
        move(0, -forwardToGetStone, 0.5); //move back (can be cut out) //(x + obs. dist. x, 0)
        */
        int dontRunIntoWall = 1;
        if (!secondTilePath) { //MOVING BACK AFTER GETTING SKYSTONE
            moveAddTolerance(0, -forwardToGetStone + MIDDLE_OF_TILE + dontRunIntoWall, MAX_POWER, .2);  // (x + obs. dist. x, middle of tile)
        } else {
            moveAddTolerance(0, -forwardToGetStone + MIDDLE_OF_TILE + dontRunIntoWall + secondTile, MAX_POWER, .2);  // (x + obs. dist. x, middle of tile)
        }
        double alignToFoundationEdge = TILE_LENGTH - ROBOT_LENGTH - FOUNDATION_AWAY_FROM_WALL;
        turnAbsolute(factor * -90);
        // turnRelative(-90 * factor);
        move(0 , TILE_LENGTH * 5 - inchesMoved - observingDistanceX + alignToFoundationEdge, MAX_POWER); //(move through alliance bridge // (5 tiles + alignToFoundationEdge, middle of tile)
        turnAbsolute(0);
        // turnRelative(90 * factor);

        extendSlidesBy(6, 0.5); //move slides up to be able to go close to foudndation

        //move(TILE_LENGTH * 2 - ROBOT_LENGTH, 0, 0.5); //move to foundation // (6 tiles, tile - robot length)
        // TODO: change 2 if need be
        //move(0, TILE_LENGTH * 2/* - ROBOT_LENGTH TODO see if this stays*//* - forwardToGetStone*/ - 2 - MIDDLE_OF_TILE, MAX_POWER); //move to foundation // (5 tiles + alignToFoundationEdge, 2 tiles - robot length - 2)
        if (!secondTilePath) {
            move(0, TILE_LENGTH * 2 - ROBOT_LENGTH - MIDDLE_OF_TILE + offsetForFoundation, MAX_POWER); // (5 tiles + alignToFoundationEdge, 2 tiles - robot length)
        } else {
            move(0, TILE_LENGTH * 2 - ROBOT_LENGTH - MIDDLE_OF_TILE + offsetForFoundation - secondTile, .5); // (5 tiles + alignToFoundationEdge, 2 tiles - robot length)
        }
        releaseStone(); //drop stone out

        //moving foundation

        //moving forwards & backwards so corner of robot doesn't hit foundation
        move(0, -6, .5); // (5 tiles + alignToFoundationEdge, 2 tiles - robot length - 6 inches)
        turnAbsolute(180); //turn around
        //tile length - robot length - found. away from wall: aligns robot to the very edge of the foundation
        moveAddTolerance(0, -6 - offsetForFoundation, .5, .2); // (5 tiles + alignToFoundationEdge, 2 tiles - robot length)
        toggleHook(true); //grab foundation

        //moving robot away from any edge to try to stop conflicts from foundation turning, magic number away: 8

        //move to wall // (6 tiles - robot length - foundation width - 8, 8)
        //TODO: 19 is a sketchy ass value

        moveAddTolerance(0, 19 + TILE_LENGTH * 2 - ROBOT_LENGTH - 8 - offsetForFoundation * 2, .5, .2);
        moveAddTolerance(factor * (-FOUNDATION_AWAY_FROM_WALL + 8 + FOUNDATION_LENGTH), 0, .5, .2);

        //moveAddTolerance(factor * (-FOUNDATION_AWAY_FROM_WALL + 8 + FOUNDATION_WIDTH), 19 + TILE_LENGTH * 2 - ROBOT_LENGTH - 8 - offsetForFoundation, 0.5, .2);

        turnAbsolute(180 - 90 * factor);
        // turnRelative(factor * -90);
        //moving foundation all the way to corner
        moveAddTolerance(factor * -8, 0, .5, .2);
        moveAddTolerance(0, -8, .5, .2);
        //moveAddTolerance( factor * -8, -8, .5, .2); // (6 tiles - robot length - foundation width, 0)
        toggleHook(false);
        if (!secondTilePath) {
            move(factor * MIDDLE_OF_TILE, 0, .5); //6 tiles - robot length - foundation width, middle of tile)
        } else {
            move(factor * (MIDDLE_OF_TILE + secondTile), 0, .5); //6 tiles - robot length - foundation width, middle of tile)
        }
        double tempPosition = 5 * TILE_LENGTH - ROBOT_LENGTH - FOUNDATION_LENGTH;
        extendSlidesBy(-6, 0.5); //move slides back down

        if (!second) { //if don't want second block
            telemetry.addData("1", "not second block");
            telemetry.update();

            // robot currently facing sideways
            // middle of robot will hopefully be on tape this way
            move(0, tempPosition - (3 * TILE_LENGTH - ROBOT_LENGTH / 2), MAX_POWER); //parks on tape // (3 tiles - half of robot length, middle of tile)

        } else { // if want second block
            telemetry.addData("2", "second block");
            telemetry.update();

            //to try to get the second block
            move(0, tempPosition - 3 * BLOCK_LENGTH - observingDistanceX, MAX_POWER); //move to second set of blocks // (3 blocks + obs. dist. x, middle of tile)
            turnAbsolute(0); //turn back forwards
            if (!secondTilePath) {
                move(0, forwardToGetStone - MIDDLE_OF_TILE, MAX_POWER); //move forward to block // (3 blocks + obs. dist. x, forwardToGetStone)
            } else {
                move(0, forwardToGetStone - MIDDLE_OF_TILE - secondTile, MAX_POWER); //move forward to block // (3 blocks + obs. dist. x, forwardToGetStone)

            }
            inchesMoved = grabSkystone(color); //grabbed block // (3 blocks + x, robot length)
            if (!secondTilePath) {
                move(0, -forwardToGetStone + MIDDLE_OF_TILE, MAX_POWER); //move back // (3 blocks + x + obs. dist. x, middle of tile)
            } else {
                move(0, -forwardToGetStone + MIDDLE_OF_TILE + secondTile, MAX_POWER); //move back // (3 blocks + x + obs. dist. x, middle of tile)
            }
            turnAbsolute(-90 * factor); //turn towards foundation, then move forwards

            //move slides up to be able to move close to foundation to drop
            extendSlidesBy(6, .5);

            //moving to the edge of the foundation
            // (6 blocks - foundation  - robot length, middle of tile)
            move(0, 6 * TILE_LENGTH - FOUNDATION_LENGTH - ROBOT_LENGTH - (BLOCK_LENGTH * 3 + inchesMoved + observingDistanceX), MAX_POWER);

            releaseStone();

            extendSlidesBy(-6, 0.5);

            //moving backwards towards tape
            move(0, -1 * (TILE_LENGTH * 6 - FOUNDATION_LENGTH - ROBOT_LENGTH - (3 * TILE_LENGTH - ROBOT_LENGTH / 2)), MAX_POWER); // (3 blocks - half of robot length, tile length);
        }
    }

    public void bridgeAuto(String color) { // we shouldn't get second block for bridge autonomous, will not move foundation
        int factor;
        if (color.equals("red"))
            factor = 1;
        else
            factor = -1;

        bringSlidesDown();

        double secondTile = TILE_LENGTH;

        double observingDistance = 10;
        double observingDistanceX = observingDistance / Math.sqrt(2);
        double observingDistanceY = observingDistance / Math.sqrt(2);

        double forwardToGetStone = 2 * TILE_LENGTH - ROBOT_LENGTH - observingDistanceY;

        // (2 tiles, 0)
        move(-factor * (2 * TILE_LENGTH - 3 * BLOCK_LENGTH - observingDistanceX), 0, .5); // (3 blocks + obs. dist. x, 0)
        move(0, forwardToGetStone, .5); // (3 blocks + obs. dist. x, forwardToGetStone)
        int inchesMoved = grabSkystone(color); // (3 blocks + obs. dist. x + x, forwardToGetStone)

        move(0, TILE_LENGTH + MIDDLE_OF_TILE - forwardToGetStone, .5);
        turnAbsolute(factor * -90);
        move(0, 4 * TILE_LENGTH + TILE_LENGTH / 2 - (3 * BLOCK_LENGTH + observingDistanceX + inchesMoved), .5); // (4.5 tiles, forwardToGetStone)

        extendSlidesBy(8, 0.5);
        turnAbsolute(0);
        double moveForFoundation = 5;
        move(0, moveForFoundation + MIDDLE_OF_TILE, .5); // (4.5 tiles, 2 tiles - robot length)
        releaseStone();
        //moves robot to the middle of the second tile
        move(0, -1 * moveForFoundation - MIDDLE_OF_TILE, .5); // (4.5 tiles, centered on second tile)
        extendSlidesBy(-8, 0.5);

        turnAbsolute(factor * 90);
        move(0, 4.5 * TILE_LENGTH - 3 * TILE_LENGTH + ROBOT_LENGTH / 2, .5); // (3 tiles - half robot, centered on second tile)

    }

    public void foundationAuto(int milliseconds, String color){
        int factor;
        if (color.equals("red"))
            factor = 1;
        else
            factor = -1;

        double MAX_POWER = 0.5;

        //sleep(9*1000);

        double smashIntoFoundation = 1;
        double slowDownPos = 0;
        move(0, -47.25+ ROBOT_LENGTH - smashIntoFoundation + slowDownPos, .3);
        move(0,-slowDownPos, 0.2);
        sleep(500);
        toggleHook(true);
        sleep(500);

        move(0,TILE_LENGTH + 1, MAX_POWER);

        turnAbsolute(factor * -90);
        bringSlidesDown();
        sleep(500);
        //moveSetISetP(0,-10,0.5, 0.75, 0.01, 1.0/600);
        move(0,-10);

        toggleHook(false);

        move(factor * (TILE_LENGTH / 4),0, MAX_POWER);
        move(0, 1.25 * TILE_LENGTH + 6, MAX_POWER);
    }

    public void tileSidewaysForwards(String direction) { //STARTS ON THE MIDDLE OF THIRD TILE FROM THE SKYSTONES
        int factor;
        if (direction.equals("right"))
            factor = 1;
        else
            factor = -1;

        bringSlidesDown();
        outwardsIntake();
        move(0, TILE_LENGTH + MIDDLE_OF_TILE - 2, .5);
        tileSideways(direction);
    }
    public void tileSideways(String direction) {
        int factor;
        if (direction.equals("right"))
            factor = 1;
        else
            factor = -1;

        //bringSlidesDown();
        move(factor * (TILE_LENGTH / 2), 0, .5);
        //outwardsIntake();
        extendSlidesBy(2,0.5);
        sleep(500);
        outwardsIntake();
        sleep(500);
        extendSlidesTo(slideMin, .5);
    }
}
