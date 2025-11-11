package org.firstinspires.ftc.teamcode.Autonomous;

//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.pedropathing.ftc.localization.Encoder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;



import org.firstinspires.ftc.teamcode.ComplexRobots.FirstAgeTempbot;

@Autonomous
public class smallTimedPedro extends LinearOpMode {

    FirstAgeTempbot robot;

    //encoder tracks motor pos, set it to 0
    public Encoder par0;

    //assigns directions to enum
    enum Direction {FORWARD, BACKWARD, LEFT, RIGHT, POSITIVE, NEGATIVE}

    @Override
    public void runOpMode() throws InterruptedException {
        //creates new object for robot: includes position, vectors, and map)
        robot = new FirstAgeTempbot(hardwareMap, new Pose2d(new Vector2d(0,0),0), this);  //idk whats wrong here pls fix it

        //brakes aka sets all mp to 0
        robot.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //robot waits patiently
        waitForStart();

        //setting time allotted for each action, how much motor power to use, and sets startTime to the current time
        rotateTo(Direction.POSITIVE, 1000, System.currentTimeMillis(), 0.5);
        driveAllMotorsTo(Direction.FORWARD, 2000, System.currentTimeMillis(), 0.5);
        rotateTo(Direction.NEGATIVE, 1000, System.currentTimeMillis(), 0.5);
        driveAllMotorsTo(Direction.FORWARD, 6000, System.currentTimeMillis(), 0.5);
//        robot.specimenClaw.setPosition(0.5);

    }



    //defines what to do for each movement: moves in that direction for the time allotted for that action (millisDelay), sets the motor powers to variable defined above
    public void strafeMotorsTo(Direction direction, int millisDelay, long startTime, double motorPower){
        if(direction == Direction.LEFT){
            while((System.currentTimeMillis() - startTime) < millisDelay){
                robot.setMotorPower(-motorPower, motorPower, motorPower, -motorPower);
            }
        } else if(direction == Direction.RIGHT){
            while((System.currentTimeMillis() - startTime) < millisDelay){
                robot.setMotorPower(motorPower, -motorPower, -motorPower, motorPower);
            }
        }
    }

    //sets y direction for the robot, moves in that direction for the time allotted for that action (millisDelay), sets the motor powers to variable defined above
    public void driveAllMotorsTo(Direction direction, int millisDelay, long startTime, double motorPower){
        if(direction == Direction.FORWARD){
            while((System.currentTimeMillis() - startTime) < millisDelay){
                robot.setAllMotorPowers(motorPower);
            }
        } else if(direction == Direction.BACKWARD){
            while((System.currentTimeMillis() - startTime) < millisDelay){
                robot.setAllMotorPowers(-motorPower);
            }
        }
    }

    public void rotateTo(Direction direction, int millisDelay, long startTime, double motorPower){
        robot.setAllMotorPowers(motorPower);
        if(direction == Direction.POSITIVE){
            while((System.currentTimeMillis() - startTime) < millisDelay){
                robot.setMotorPower(-motorPower, motorPower, -motorPower, motorPower);
            }
        } else if(direction == Direction.NEGATIVE){
                while((System.currentTimeMillis() - startTime) < millisDelay){
                    robot.setMotorPower(motorPower, -motorPower, motorPower, -motorPower);
                }
            }
        }


}

