package org.firstinspires.ftc.teamcode.Autonomous.TimeBased;

//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import static org.firstinspires.ftc.teamcode.Autonomous.TimeBased.TimeAutoFunctions.driveAllMotorsTo;
import static org.firstinspires.ftc.teamcode.Autonomous.TimeBased.TimeAutoFunctions.rotateTo;
import static org.firstinspires.ftc.teamcode.Autonomous.TimeBased.TimeAutoFunctions.strafeMotorsTo;
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

@Autonomous(name = "triangleTimedRed")
public class triangleTimedRed extends LinearOpMode {

    ServoTempBot robot;
    public static double baseShotPower = .45;
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
        limelightData.ImuOffset = Math.PI / 2 - 2.65;
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
        driveAllMotorsTo(GoalTimedRed.Direction.FORWARD, 220, System.currentTimeMillis(), .8, robot);
        robot.setMotorPower(0,0,0,0);
        sleep(500);
        strafeMotorsTo(GoalTimedRed.Direction.LEFT, 506, System.currentTimeMillis(), .8, robot);
        robot.setMotorPower(0,0,0,0);
        sleep(500);
        rotateTo(GoalTimedRed.Direction.NEGATIVE, 225, System.currentTimeMillis(), .5, robot);
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
                robot.Servo2.setPosition(-.2);
            }
            stage = 1;
            if (timeSince + timeBuffer2 < System.currentTimeMillis() && timeSince + 6500 > System.currentTimeMillis()) {
                robot.Servo2.setPosition(.2);
                robot.Servo1.setPower(-.1);
            }
            else if (timeSince + timeBuffer < System.currentTimeMillis()) {
                robot.Servo1.setPower(-.5);
//                robot.Servo3.setPower(.5);
                robot.Servo2.setPosition(-.3);
                telemetry.addLine("Servos");
            }


            telemetry.addLine("Shoot limelight");
            telemetry.update();
            //This is meant to shoot according to the distance to the april tag if the limelight is accurate
            //All of these variables are yet to be tested and should be iterated on
            double targetVel = -FirstAgeArm.velocityShot(195);
            double shootVel = robot.ShootMotor.getVelocity();

            ShootingFunctions.setVelocity(targetVel, shootVel, robot.ShootMotor, 1);

        }
        robot.ShootMotor.setPower(0);
        robot.Servo2.setPosition(.7);
        rotateTo(GoalTimedRed.Direction.POSITIVE, 242, System.currentTimeMillis(), .5, robot);
        strafeMotorsTo(GoalTimedRed.Direction.RIGHT, 556, System.currentTimeMillis(), .8, robot);
//        sleep(1000);
//        aim( 180,50, 1, robot);
//        sleep(1000);
//        aim( 45,50, 1, robot);

//        driveAllMotorsTo(Direction.BACKWARD, 6000, System.currentTimeMillis(), 0.5);
    }


    //defines what to do for each movement: moves in that direction for the time allotted for that action (millisDelay), sets the motor powers to variable defined above

}
    

