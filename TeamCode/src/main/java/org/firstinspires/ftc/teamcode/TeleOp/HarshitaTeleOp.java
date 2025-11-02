package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ButtonMaps.AbstractButtonMap;
import org.firstinspires.ftc.teamcode.ButtonMaps.Arm.FirstAgeArm;
import org.firstinspires.ftc.teamcode.ButtonMaps.Drive.HarshitaBM;
import org.firstinspires.ftc.teamcode.ButtonMaps.ServoAbstractButtonMap;
import org.firstinspires.ftc.teamcode.ComplexRobots.FirstAgeTempbot;
import org.firstinspires.ftc.teamcode.ComplexRobots.ServoTempBot;

@TeleOp(name="Harshita TeleOp")
public class HarshitaTeleOp extends OpMode {
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
        driveButtonMap = new HarshitaBM();
        armButtonmap = new FirstAgeArm();
        telemetry.addLine("Ready.");
        telemetry.update();
    }

    @Override
    public void loop() {
        driveButtonMap.loop(robot, this);
        armButtonmap.loop(robot, this);
        telemetry.update();
    }
}
