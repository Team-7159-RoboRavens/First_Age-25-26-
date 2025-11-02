package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ButtonMaps.AbstractButtonMap;
import org.firstinspires.ftc.teamcode.ButtonMaps.Arm.FirstAgeArm;
import org.firstinspires.ftc.teamcode.ButtonMaps.Drive.TempBotDrive;
import org.firstinspires.ftc.teamcode.ComplexRobots.FirstAgeTempbot;

@TeleOp(name="TempBot TeleOp Blue")
public class TempbotTeleOpBlue extends OpMode {
    //Global Variables
    FirstAgeTempbot robot;

    //Button Maps
    AbstractButtonMap driveButtonMap;

    @Override
    public void init() {
        telemetry.addLine("Initializing, please wait...");
        telemetry.update();
        robot = new FirstAgeTempbot(hardwareMap, new Pose2d(0,0,0), this);
        driveButtonMap = new TempBotDrive();
        telemetry.addLine("Ready.");
        telemetry.update();
    }

    //This code runs through the other files on a clock to respond to inputs.
    @Override
    public void loop() {
        driveButtonMap.loop(robot, this);
        //This should be the april tag id for the blue tower.
        robot.runLimelight(25);
        telemetry.update();
    }
}
