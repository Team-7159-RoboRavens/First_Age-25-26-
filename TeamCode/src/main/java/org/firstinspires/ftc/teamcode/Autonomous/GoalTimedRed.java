package org.firstinspires.ftc.teamcode.Autonomous;

//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import static org.firstinspires.ftc.teamcode.ButtonMaps.Arm.FirstAgeArm.velocityShot;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.pedropathing.ftc.localization.Encoder;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.ComplexRobots.ServoTempBot;
import org.firstinspires.ftc.teamcode.limelightData;

@Autonomous(name = "GoalTImedRed")
public class GoalTimedRed extends LinearOpMode {

    ServoTempBot robot;
    public static double baseShotPower = .418;
    public static double limelightPowerMultiplier = 1.18 ;
    public static double limelightBaseDistance = 100;
    public static double nonLinearPower = 1.0028;
    private static double onSpeed = 0;


    //encoder tracks motor pos, set it to 0
    public Encoder par0;

    //assigns directions to enum
    enum Direction {FORWARD, BACKWARD, LEFT, RIGHT, POSITIVE, NEGATIVE}

    @Override
    public void runOpMode() throws InterruptedException {
        //creates new object for robot: includes position, vectors, and map)
        limelightData.hasImu = false;
        robot = new ServoTempBot(hardwareMap, new Pose2d(new Vector2d(0, 0), 0), this);  //idk whats wrong here pls fix it
//        robot.lazyImu = new LazyImu(hardwareMap, "imu", new RevHubOrientationOnRobot(
//                new Orientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES, -180, 0, 0, 0)));

        limelightData.ImuOffset = Math.PI - 2.08;
        //brakes aka sets all mp to 0
        robot.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.ShootMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //robot waits patiently
        waitForStart();

//        //setting time allotted for each action, how much motor power to use, and sets startTime to the current time
//        strafeMotorsTo(Direction.LEFT, 50, System.currentTimeMillis(), .5);
        robot.Servo2.setPosition(.7);
//        aim( -20,.1, .3, robot, telemetry);
//        telemetry.addLine("Aiming");
//        telemetry.update();
//        sleep(500);
        driveAllMotorsTo(Direction.FORWARD, 900, System.currentTimeMillis(), .8);
        robot.setMotorPower(0,0,0,0);
        sleep(500);
//        strafeMotorsTo(GoalTimedRed.Direction.RIGHT, 300, System.currentTimeMillis(), .8);
//        robot.setMotorPower(0,0,0,0);
//        sleep(500);
        robot.setMotorPower(0,0,0,0);
        rotateTo(Direction.POSITIVE, 1350, System.currentTimeMillis(), .5);
//        time
//        while () {
//
//        }
        telemetry.addLine("rotating");
        telemetry.update();
//        robot.runLimelight(24);
//        aimLimelight(robot);

        long timeSince = System.currentTimeMillis();
        int stage = 0;
        double timeBuffer = 5000;
        double timeBuffer2 = 6900;
        double timeSet = System.currentTimeMillis();
        while (System.currentTimeMillis() < timeSet + 17000) {
            double targetVel = velocityShot(140);
            double shootVel = robot.ShootMotor.getVelocity();
            if (Math.abs(targetVel - shootVel) <= 20) {
                robot.ShootMotor.setPower(onSpeed);
            }
            else {
                onSpeed = (targetVel - shootVel) / 60;
                robot.ShootMotor.setPower((targetVel - shootVel) / 60);
            }
            telemetry.addData("676767 ", shootVel);
            telemetry.addData("target velocity = ", targetVel);

            if (stage == 0) {
                timeSet = System.currentTimeMillis();
                timeSince = System.currentTimeMillis();
                robot.Servo2.setPosition(.7);
            }
            stage = 1;
            if (timeSince + timeBuffer2 < System.currentTimeMillis() && timeSince + 9000 > System.currentTimeMillis()) {
                robot.Servo2.setPosition(.7);
                robot.Servo1.setPower(-.07);
            }
            else if (timeSince + timeBuffer < System.currentTimeMillis()) {
                robot.Servo1.setPower(-.7);
                robot.Servo3.setPower(.8);
                robot.Servo2.setPosition(.4);
                telemetry.addLine("Servos");
            }


            telemetry.addLine("Shoot limelight");
            telemetry.update();
            //This is meant to shoot according to the distance to the april tag if the limelight is accurate
            //All of these variables are yet to be tested and should be iterated on
//            if (!limelightData.accurate)
//                telemetry.addLine("Shoot far");

        }
        robot.ShootMotor.setPower(0);
        robot.Servo2.setPosition(.7);
        robot.Servo3.setPower(0);
        driveAllMotorsTo(GoalTimedRed.Direction.FORWARD, 600, System.currentTimeMillis(), .8);
        robot.setMotorPower(0,0,0,0);
        rotateTo(Direction.NEGATIVE, 600, System.currentTimeMillis(), .5);

//        driveAllMotorsTo(Direction., 400, System.currentTimeMillis(), .5);

//        driveAllMotorsTo(GoalTimedRed.Direction.FORWARD, 800, System.currentTimeMillis(), .6);
//        sleep(500);
//        sleep(1000);
//        aim( 180,50, 1, robot);
//        sleep(1000);
//        aim( 45,50, 1, robot);

//        driveAllMotorsTo(Direction.BACKWARD, 6000, System.currentTimeMillis(), 0.5);
    }


    //defines what to do for each movement: moves in that direction for the time allotted for that action (millisDelay), sets the motor powers to variable defined above
    public void strafeMotorsTo(Direction direction, int millisDelay, long startTime, double motorPower) {
        if (direction == Direction.LEFT) {
            while ((System.currentTimeMillis() - startTime) < millisDelay) {
                robot.setMotorPower(-motorPower, motorPower, motorPower, -motorPower);
            }
        } else if (direction == Direction.RIGHT) {
            while ((System.currentTimeMillis() - startTime) < millisDelay) {
                robot.setMotorPower(motorPower, -motorPower, -motorPower, motorPower);
            }
        }
        robot.setMotorPower(0,0,0,0);
    }

    //sets y direction for the robot, moves in that direction for the time allotted for that action (millisDelay), sets the motor powers to variable defined above
    public void driveAllMotorsTo(Direction direction, int millisDelay, long startTime, double motorPower) {
        if (direction == Direction.FORWARD) {
            while ((System.currentTimeMillis() - startTime) < millisDelay) {
                robot.setAllMotorPowers(motorPower);
            }
        } else if (direction == Direction.BACKWARD) {
            while ((System.currentTimeMillis() - startTime) < millisDelay) {
                robot.setAllMotorPowers(-motorPower);
            }
        }
        robot.setMotorPower(0,0,0,0);
    }

    //negative is right, positive is left
    public void rotateTo(Direction direction, int millisDelay, long startTime, double motorPower) {
//        robot.setAllMotorPowers(motorPower);
        if (direction == Direction.POSITIVE) {
            while ((System.currentTimeMillis() - startTime) < millisDelay) {
                robot.setMotorPower(-motorPower, motorPower, -motorPower, motorPower);
            }
        } else if (direction == Direction.NEGATIVE) {
            while ((System.currentTimeMillis() - startTime) < millisDelay) {
                robot.setMotorPower(motorPower, -motorPower, motorPower, -motorPower);
            }
        }
        robot.setMotorPower(0,0,0,0);

    }

    public static void aim(double desiredAngle, double millisDelay, double motorPower, ServoTempBot robot, Telemetry telemetry) {
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
            target -= 360;
        }
//        aim(target, 50,  1, robot2,);
    }
    static public void timeout(double input) {
        double startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() < startTime + input) {
            System.out.println(System.currentTimeMillis());
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
    

