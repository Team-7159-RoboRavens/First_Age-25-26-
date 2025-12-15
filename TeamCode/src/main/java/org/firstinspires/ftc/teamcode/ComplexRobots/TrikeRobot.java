package org.firstinspires.ftc.teamcode.ComplexRobots;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ButtonMaps.MotorPowers;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TrikeDrive;

//This robot, as it sounds, is specifically for the trike. It is only for a bot with one motor and one servo as its only control.

@Config
public class TrikeRobot extends TrikeDrive {
    enum Direction {
        UP,DOWNw
    }

    public final Servo turnServo;


    public TrikeRobot(HardwareMap hardwareMap, Pose2d pose) {
        super(hardwareMap, pose);

        //Servos
        turnServo = hardwareMap.get(Servo.class, "turnServo");

        //Initialize Output Servo
        turnServo.scaleRange(-1,1);
    }


    //TODO: Linear slide helper methods for auto (later)
    public MotorPowers setAllMotorPowers(int i) {
        return new MotorPowers(0,0,0,0);
    }

    //This is meant mostly for slides to extent or run a motor to a certain point.

    public void setMotorTo(DcMotorEx motor, int targetPos, double power) {
        if (motor.getCurrentPosition() < targetPos) {
            while (motor.getCurrentPosition() <= targetPos) {
                motor.setPower(power);
            }
        }
        else if (motor.getCurrentPosition() > targetPos) {
            while (motor.getCurrentPosition() >= targetPos) {
                motor.setPower(-power);
            }
        }
    }
    }
