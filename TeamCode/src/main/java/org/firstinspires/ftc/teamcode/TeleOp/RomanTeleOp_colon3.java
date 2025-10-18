package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ButtonMaps.AbstractButtonMap;
import org.firstinspires.ftc.teamcode.ButtonMaps.Arm.FirstAgeArm;
import org.firstinspires.ftc.teamcode.ButtonMaps.Drive.RomanDrive;
import org.firstinspires.ftc.teamcode.ComplexRobots.FirstAgeTempbot;

@TeleOp(name="Roman TeleOp :3")
public class RomanTeleOp_colon3 extends OpMode {
    //Global Variables
    FirstAgeTempbot robot;

    //Button Maps
    AbstractButtonMap driveButtonMap;
    AbstractButtonMap armButtonMap;

    @Override
    public void init() {
        telemetry.addLine("Initializing, please wait...");
        telemetry.update();
        robot = new FirstAgeTempbot(hardwareMap, new Pose2d(0,0,0));
        driveButtonMap = new RomanDrive();
        armButtonMap = new FirstAgeArm();
        telemetry.addLine("Ready!");
        telemetry.update();
    }

    @Override
    public void loop() {
        driveButtonMap.loop(robot, this);
        armButtonMap.loop(robot, this);
        telemetry.update();
    }
}
