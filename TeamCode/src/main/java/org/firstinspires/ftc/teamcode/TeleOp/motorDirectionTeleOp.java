package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ButtonMaps.Drive.motorDirectionDebugger;
import org.firstinspires.ftc.teamcode.ButtonMaps.WheelTestAbstractButtonMap;
import org.firstinspires.ftc.teamcode.ComplexRobots.WheelTestBot;

@TeleOp(name="Motor Direction Debugger")
public class motorDirectionTeleOp extends OpMode {
    //Global Variables
    WheelTestBot robot;

    //Button Maps
    WheelTestAbstractButtonMap driveButtonMap;

    @Override
    public void init() {
        telemetry.addLine("Initializing, please wait...");
        telemetry.update();
        robot = new WheelTestBot(hardwareMap, new Pose2d(0,0,0), this);
        driveButtonMap = new motorDirectionDebugger();
        telemetry.addLine("Ready.");
        telemetry.update();
    }

    @Override
    public void loop() {
        driveButtonMap.loop(robot, this);
        telemetry.update();
    }
}
