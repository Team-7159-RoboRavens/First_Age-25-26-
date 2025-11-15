package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ButtonMaps.Arm.FirstAgeArm;
import org.firstinspires.ftc.teamcode.ButtonMaps.Drive.LiamPolarDrive;
import org.firstinspires.ftc.teamcode.ButtonMaps.ServoAbstractButtonMap;
import org.firstinspires.ftc.teamcode.ComplexRobots.ServoTempBot;

@TeleOp(name="LM1TeleopRed")
public class LM1TeleopRed extends OpMode {
    //Global Variables
    ServoTempBot robot;

    //Button Maps
    ServoAbstractButtonMap driveButtonMap;
    ServoAbstractButtonMap armButtonmap;

    @Override
    public void init() {
        telemetry.addLine("Initializing, please wait...");
        telemetry.update();
        robot = new ServoTempBot(hardwareMap, new Pose2d(0,0,0), this);
        driveButtonMap = new LiamPolarDrive();
        armButtonmap = new FirstAgeArm();
        telemetry.addLine("Ready.");
        telemetry.update();
    }

    @Override
    public void loop() {
        driveButtonMap.loop(robot, this);
        armButtonmap.loop(robot, this);
        robot.runLimelight(24);
        telemetry.update();
    }
}
