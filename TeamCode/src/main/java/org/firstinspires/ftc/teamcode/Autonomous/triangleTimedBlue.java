package org.firstinspires.ftc.teamcode.Autonomous;

//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import static org.firstinspires.ftc.teamcode.ButtonMaps.Arm.FirstAgeArm.velocityShot;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.pedropathing.ftc.localization.Encoder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.ComplexRobots.ServoTempBot;
import org.firstinspires.ftc.teamcode.limelightData;

@Autonomous(name = "triangleTimedBlue")
public class triangleTimedBlue extends LinearOpMode {

    ServoTempBot robot;
    public static double baseShotPower = .438;
    public static double limelightPowerMultiplier = 1.18 ;
    public static double limelightBaseDistance = 100;
    public static double nonLinearPower = 1.0028;
    public static double onSpeed = 0;


    //encoder tracks motor pos, set it to 0
    public Encoder par0;

    //assigns directions to enum
    enum Direction {FORWARD, BACKWARD, LEFT, RIGHT, POSITIVE, NEGATIVE}

    @Override
    public void runOpMode() throws InterruptedException {
        //creates new object for robot: includes position, vectors, and map)
        robot = new ServoTempBot(hardwareMap, new Pose2d(new Vector2d(0, 0), 0), this);  //idk whats wrong here pls fix it
        limelightData.ImuOffset = Math.PI / 2;
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
        driveAllMotorsTo(Direction.FORWARD, 220, System.currentTimeMillis(), .8);
        robot.setMotorPower(0,0,0,0);
        sleep(500);
        strafeMotorsTo(Direction.RIGHT, 506, System.currentTimeMillis(), .8);
        robot.setMotorPower(0,0,0,0);
        sleep(500);
        rotateTo(Direction.POSITIVE, 60, System.currentTimeMillis(), .5);
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
        double timeBuffer = 3000;
        double timeBuffer2 = 4600;
        double timeSet = System.currentTimeMillis();
        while (System.currentTimeMillis() < timeSet + 17000) {
            if (stage == 0) {
                timeSet = System.currentTimeMillis();
                timeSince = System.currentTimeMillis();
                robot.Servo2.setPosition(.7);
            }
            stage = 1;
            if (timeSince + timeBuffer2 < System.currentTimeMillis() && timeSince + 6500 > System.currentTimeMillis()) {
                robot.Servo2.setPosition(.7);
                robot.Servo1.setPower(-.1);
            }
            else if (timeSince + timeBuffer < System.currentTimeMillis()) {
                robot.Servo1.setPower(-.5);
                robot.Servo3.setPower(.5);
                robot.Servo2.setPosition(.4);
                telemetry.addLine("Servos");
            }


            telemetry.addLine("Shoot limelight");
            telemetry.update();
            //This is meant to shoot according to the distance to the april tag if the limelight is accurate
            //All of these variables are yet to be tested and should be iterated on
            double targetVel = velocityShot(195);
            double shootVel = robot.ShootMotor.getVelocity();

//            if (Math.abs(targetVel - shootVel) <= 20) {
//                robot.ShootMotor.setPower(onSpeed);
//            }
//            else {
                onSpeed = (targetVel - shootVel) / 60;
                robot.ShootMotor.setPower((targetVel - shootVel) / 100);
//            }
//            if (!limelightData.accurate)
//                telemetry.addLine("Shoot far");

        }
        robot.ShootMotor.setPower(0);
        robot.Servo2.setPosition(.7);
        rotateTo(Direction.NEGATIVE, 142, System.currentTimeMillis(), .5);
        strafeMotorsTo(Direction.LEFT, 550, System.currentTimeMillis(), .8);
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
        double angleOffset = Math.min(Math.abs(currentAngle - desiredAngle), Math.abs(currentAngle + desiredAngle));
        telemetry.addData("Angle Offset ", angleOffset);
        telemetry.update();
        while (angleOffset - millisDelay * 1 > 0) {
            currentAngle = robot.lazyImu.get().getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            angleOffset = Math.min(Math.abs(currentAngle - desiredAngle), Math.abs(currentAngle + desiredAngle));
            boolean rotateRight = false;
            if ((currentAngle - desiredAngle) < (currentAngle + desiredAngle)) {
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


