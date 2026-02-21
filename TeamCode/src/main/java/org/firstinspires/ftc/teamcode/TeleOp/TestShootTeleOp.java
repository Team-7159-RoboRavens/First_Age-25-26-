package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ButtonMaps.Arm.GoodArmNoWheels;
import org.firstinspires.ftc.teamcode.ButtonMaps.Arm.buttonTester;
import org.firstinspires.ftc.teamcode.ButtonMaps.NoWheelsAbstractButtonMap;
import org.firstinspires.ftc.teamcode.ComplexRobots.ShootOnlyBot;

@TeleOp(name="Test Double Shoot Motors")
public class TestShootTeleOp extends OpMode {
    //Global Variables
    ShootOnlyBot robot;

    //Button Maps
    NoWheelsAbstractButtonMap driveButtonMap;

    @Override
    public void init() {
        telemetry.addLine("Initializing, please wait...");
        telemetry.update();
        robot = new ShootOnlyBot(hardwareMap, new Pose2d(0,0,0), this);
        driveButtonMap = new GoodArmNoWheels();
        telemetry.addLine("Ready.");
        telemetry.update();
    }

    @Override
    public void loop() {
        driveButtonMap.loop(robot, this);
        telemetry.update();
    }
}
