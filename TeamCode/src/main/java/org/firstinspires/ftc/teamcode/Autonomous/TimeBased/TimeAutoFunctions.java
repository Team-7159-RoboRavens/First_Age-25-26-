package org.firstinspires.ftc.teamcode.Autonomous.TimeBased;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.ComplexRobots.ServoTempBot;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.limelightData;

public class TimeAutoFunctions {


    public static void strafeMotorsTo(GoalTimed.Direction direction, int millisDelay, long startTime, double motorPower, MecanumDrive robot) {
        if (direction == GoalTimed.Direction.LEFT) {
            while ((System.currentTimeMillis() - startTime) < millisDelay) {
                robot.setMotorPower(motorPower, -motorPower, -motorPower, motorPower);
            }
        } else if (direction == GoalTimed.Direction.RIGHT) {
            while ((System.currentTimeMillis() - startTime) < millisDelay) {
                robot.setMotorPower(-motorPower, motorPower, motorPower, -motorPower);
            }
        }
        robot.setMotorPower(0,0,0,0);
    }

    //sets y direction for the robot, moves in that direction for the time allotted for that action (millisDelay), sets the motor powers to variable defined above
    public static void driveAllMotorsTo(GoalTimed.Direction direction, int millisDelay, long startTime, double motorPower, MecanumDrive robot) {
        if (direction == GoalTimed.Direction.FORWARD) {
            while ((System.currentTimeMillis() - startTime) < millisDelay) {
                robot.setAllMotorPowers(-motorPower);
            }
        } else if (direction == GoalTimed.Direction.BACKWARD) {
            while ((System.currentTimeMillis() - startTime) < millisDelay) {
                robot.setAllMotorPowers(motorPower);
            }
        }
        robot.setMotorPower(0,0,0,0);
    }

    //negative is right, positive is left
    public static void rotateTo(GoalTimed.Direction direction, int millisDelay, long startTime, double motorPower, MecanumDrive robot) {
//        robot.setAllMotorPowers(motorPower);
        if (direction == GoalTimed.Direction.POSITIVE) {
            while ((System.currentTimeMillis() - startTime) < millisDelay) {
                robot.setMotorPower(motorPower, -motorPower, motorPower, -motorPower);
            }
        } else if (direction == GoalTimed.Direction.NEGATIVE) {
            while ((System.currentTimeMillis() - startTime) < millisDelay) {
                robot.setMotorPower(-motorPower, motorPower, -motorPower, motorPower);
            }
        }
        robot.setMotorPower(0,0,0,0);

    }

    public static void aim(double desiredAngle, double millisDelay, double motorPower, Telemetry telemetry, MecanumDrive robot) {
        double currentAngle = robot.lazyImu.get().getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) - 180;
        telemetry.addData("Angle ", currentAngle);
//        desiredAngle -= 180;
        double rightDegrees = desiredAngle - currentAngle;
        if (rightDegrees < 0){
            rightDegrees += 180;
        }

        double leftDegrees = currentAngle - desiredAngle;
        if (leftDegrees < 0){
            leftDegrees += 180;
        }


        double angleOffset = Math.min(leftDegrees, rightDegrees);
        telemetry.addData("Angle Offset ", angleOffset);
        telemetry.update();
        while (angleOffset - millisDelay * 1 > 0) {
            currentAngle = robot.lazyImu.get().getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            boolean rotateRight = false;
            rightDegrees = desiredAngle - currentAngle;
            if (rightDegrees < 0) {
                rightDegrees += 180;
            }
            leftDegrees = currentAngle - desiredAngle;
            if (leftDegrees < 0){
                leftDegrees += 180;
            }
            angleOffset = Math.min(leftDegrees, rightDegrees);
            if (rightDegrees < leftDegrees) {
                rotateRight = true;
            }
            if (rotateRight) {
                robot.setMotorPower(-motorPower, motorPower, -motorPower, motorPower);
            } else {
                robot.setMotorPower(motorPower, -motorPower, motorPower, -motorPower);
            }
        }
        robot.setMotorPower(0,0,0,0);
    }
    public static void rotate(double degrees, ServoTempBot robot2){
        double target = robot2.lazyImu.get().getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) + degrees;
        if (target > 360){
            target += 360;
        }
    }

    public static boolean aimLimelight(ServoTempBot robot2) {
        if (limelightData.accurate) {
//            aim((robot2.lazyImu.get().getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) - 180) * limelightData.aprilXDegrees, 50, .7, robot2);
            return true;
        } else {
            return false;
        }
    }

}
