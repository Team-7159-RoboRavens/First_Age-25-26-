package org.firstinspires.ftc.teamcode.Autonomous.TimeBased;

//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import static org.firstinspires.ftc.teamcode.Autonomous.TimeBased.TimeAutoFunctions.driveAllMotorsTo;
import static org.firstinspires.ftc.teamcode.Autonomous.TimeBased.TimeAutoFunctions.rotateTo;
import static org.firstinspires.ftc.teamcode.ButtonMaps.Arm.FirstAgeArm.velocityShot;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.pedropathing.ftc.localization.Encoder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.ButtonMaps.Arm.FirstAgeArm;
import org.firstinspires.ftc.teamcode.ComplexRobots.ServoTempBot;
import org.firstinspires.ftc.teamcode.ShootingFunctions;
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
        driveAllMotorsTo(Direction.FORWARD, 900, System.currentTimeMillis(), .8, robot);
        robot.setMotorPower(0,0,0,0);
        sleep(500);
//        strafeMotorsTo(GoalTimedRed.Direction.RIGHT, 300, System.currentTimeMillis(), .8);
//        robot.setMotorPower(0,0,0,0);
//        sleep(500);
        robot.setMotorPower(0,0,0,0);
        rotateTo(Direction.POSITIVE, 1500, System.currentTimeMillis(), .5, robot);
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
            double targetVel = -FirstAgeArm.velocityShot(140);
            double shootVel = robot.ShootMotor.getVelocity();
            ShootingFunctions.setVelocity(targetVel, shootVel, robot.ShootMotor, -1);
            telemetry.addData("676767 ", shootVel);
            telemetry.addData("target velocity = ", targetVel);

            if (stage == 0) {
                timeSet = System.currentTimeMillis();
                timeSince = System.currentTimeMillis();
                robot.Servo2.setPosition(.2);
            }
            stage = 1;
            if (timeSince + timeBuffer2 < System.currentTimeMillis() && timeSince + 9000 > System.currentTimeMillis()) {
                robot.Servo2.setPosition(.2);
                robot.Servo1.setPower(-.07);
            }
            else if (timeSince + timeBuffer < System.currentTimeMillis()) {
                robot.Servo1.setPower(-.7);
//                robot.Servo3.setPower(.8);
                robot.Servo2.setPosition(-.3);
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
//        robot.Servo3.setPower(0);
        driveAllMotorsTo(Direction.FORWARD, 600, System.currentTimeMillis(), .8, robot);
        robot.setMotorPower(0,0,0,0);
        rotateTo(Direction.NEGATIVE, 600, System.currentTimeMillis(), .5, robot);

    }


}
    

