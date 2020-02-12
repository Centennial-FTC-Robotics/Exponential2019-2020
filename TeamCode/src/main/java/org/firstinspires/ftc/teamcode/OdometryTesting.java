package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class OdometryTesting extends Exponential_Methods {
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();
        double x = 0;
        double y = 0;
        while(opModeIsActive()){
            if(gamepad1.right_bumper||gamepad1.right_bumper){
                if(gamepad1.right_stick_x>0){
                    x+=.2;
                }
                if(gamepad1.right_stick_x<0){
                    x-=.2;
                }
                if(gamepad1.right_stick_y<0){
                    y+=.2;
                }
                if(gamepad1.right_stick_y>0){
                    y-=.2;
                }
            } else {
                if(gamepad1.right_stick_x>0){
                    x+=1;
                }
                if(gamepad1.right_stick_x<0){
                    x-=1;
                }
                if(gamepad1.right_stick_y<0){
                    y+=1;
                }
                if(gamepad1.right_stick_y>0){
                    y-=1;
                }
            }
            if(gamepad1.right_stick_y!=0||gamepad1.right_stick_x!=0) {
                wait(250);
            }
            if(gamepad1.a){
                move(x,y,.02,1.0/800, 0.5);
            }
        }
    }

    // rotates the point counterclockwise by angle
    private double[] rototePoint(double x, double y, double angle /*in degrees*/){
        double[] translatedPoint = new double[2];
        double angleRad = Math.PI/180*angle;
        translatedPoint[0] = x*Math.cos(angleRad)-y*Math.sin(angleRad);
        translatedPoint[1] = y*Math.cos(angleRad)+x*Math.sin(angleRad);
        return translatedPoint;
    }
    public void move(double xInch, double yInch, double pRot, double pLin, double inchesTolerence){
        double powerMax = 0.7;
        double powerMin = -0.7;

        odoWheelForwards.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odoWheelSideways.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while(odoWheelSideways.isBusy()||odoWheelForwards.isBusy()){}
        odoWheelForwards.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        odoWheelSideways.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // TODO: 2/11/2020 Find the odoForwards and odoSideways constants
        double odoForwardsError = 0; // the amount of encoders the forward odometry wheel rotates per radian of robot rotation,
        // negative indicates it loses encoder counts when rotating anti-clockwise
        double odoSidewaysError = 0; // the amount of encoders the sideways odometry wheel rotates per radian of robot rotation

        double initialAngle = getRotationinDimension('Z'); // In degrees
        // TODO: 2/11/2020  Probably have to change it so that the imu doesn't only go from -180 to 180 because that could mess stuff up
        double xTarget = convertInchToEncoderOdom(xInch); // In encoders, relative to the field, x coordinate with the origin being the start
        double yTarget = convertInchToEncoderOdom(yInch); // In encoders, relative to the field, y coordinate with the origin being the start
        double xRobotPos = 0;
        double yRobotPos = 0;

        double lastAngle = getRotationinDimension('Z'); // In degrees
        double lastodoWheelSidewaysPosition = odoWheelSideways.getCurrentPosition();
        double lastodoWheelForwardsPosition = odoWheelForwards.getCurrentPosition();

        ElapsedTime timeBetweenLoops = new ElapsedTime();
        double tolerance = convertInchToEncoderOdom(inchesTolerence); // Tolerance in encoders
        while (opModeIsActive()&&(Math.abs(xTarget-xRobotPos)>tolerance||Math.abs(yTarget-yRobotPos)>tolerance)){

            double arcDistance = Math.sqrt((odoWheelSideways.getCurrentPosition()-lastodoWheelSidewaysPosition-odoSidewaysError*(getRotationinDimension('Z')-lastAngle))
                    *(odoWheelSideways.getCurrentPosition()-lastodoWheelSidewaysPosition-odoSidewaysError*(getRotationinDimension('Z')-lastAngle))
                    +(odoWheelForwards.getCurrentPosition()-lastodoWheelForwardsPosition-odoForwardsError*(getRotationinDimension('Z')-lastAngle))
                    *(odoWheelForwards.getCurrentPosition()-lastodoWheelForwardsPosition-odoForwardsError*(getRotationinDimension('Z')-lastAngle)));
            // Since the robot rotates with the arc, the distance the odometry wheels measure is going to be the distance of the arc
            double radius = arcDistance/(Math.PI/180*(getRotationinDimension('Z')-lastAngle));

            // Segment of the arc is the chord that represents the total displacement of the robot as it travelled on the arc
            double angleOfSegment = (getRotationinDimension('Z')+lastAngle)/2;
            double distanceOfSegment= 2*radius*Math.sin(Math.PI/180*(getRotationinDimension('Z')-lastAngle)/2);


            // Rotation of the displacement to get the displacement relative to the robot
            double xDisplacement = (xTarget-xRobotPos)*Math.cos((getRotationinDimension('Z')-initialAngle)*Math.PI/180)+(yTarget-yRobotPos)*Math.sin((getRotationinDimension('Z')-initialAngle)*Math.PI/180); // Displacement is relative to robot
            double yDisplacement = -(xTarget-xRobotPos)*Math.sin((getRotationinDimension('Z')-initialAngle)*Math.PI/180)+(yTarget-yRobotPos)*Math.cos((getRotationinDimension('Z')-initialAngle)*Math.PI/180); // Displacement is relative to robot

            frontLeft.setPower(Range.clip(pLin*(yDisplacement-xDisplacement), powerMin, powerMax)-pRot*(getRotationinDimension('Z')-initialAngle));
            frontRight.setPower(Range.clip(pLin*(yDisplacement+xDisplacement), powerMin, powerMax)+pRot*(getRotationinDimension('Z')-initialAngle));
            backLeft.setPower(Range.clip(pLin*(yDisplacement+xDisplacement), powerMin, powerMax)-pRot*(getRotationinDimension('Z')-initialAngle));
            backRight.setPower(Range.clip(pLin*(yDisplacement-xDisplacement), powerMin, powerMax)+pRot*(getRotationinDimension('Z')-initialAngle));
            
            xRobotPos += distanceOfSegment*(Math.cos(angleOfSegment*Math.PI/180));
            yRobotPos += distanceOfSegment*(Math.sin(angleOfSegment*Math.PI/180));
            lastAngle = getRotationinDimension('Z');
            lastodoWheelSidewaysPosition = odoWheelSideways.getCurrentPosition();
            lastodoWheelForwardsPosition = odoWheelForwards.getCurrentPosition();
            timeBetweenLoops.reset();
        }
    }
}
