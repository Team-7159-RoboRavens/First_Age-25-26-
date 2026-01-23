package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ButtonMaps.Arm.FirstAgeGoodArm;
import org.firstinspires.ftc.teamcode.ButtonMaps.Arm.GoodArmNoWheels;
import org.firstinspires.ftc.teamcode.ButtonMaps.Drive.motorDirectionDebugger;
import org.firstinspires.ftc.teamcode.ButtonMaps.NoWheelsAbstractButtonMap;
import org.firstinspires.ftc.teamcode.ButtonMaps.ServoAbstractButtonMapGood;
import org.firstinspires.ftc.teamcode.ButtonMaps.WheelTestAbstractButtonMap;
import org.firstinspires.ftc.teamcode.ComplexRobots.ServoGoodBot;
import org.firstinspires.ftc.teamcode.ComplexRobots.ShootOnlyBot;
import org.firstinspires.ftc.teamcode.ComplexRobots.WheelTestBot;
import org.firstinspires.ftc.teamcode.FlywheelPDIFF;

@TeleOp(name="PIDFF Bot")
public class goodBotShootTeleOp extends OpMode {
    //Global Variables
    ServoGoodBot robot;

    //Button Maps
    ServoAbstractButtonMapGood driveButtonMap;
    FlywheelPDIFF armButtonMap;

    @Override
    public void init() {
        telemetry.addLine("Initializing, please wait...");
        telemetry.update();
        robot = new ServoGoodBot(hardwareMap, new Pose2d(0,0,0), this);
        driveButtonMap = new FirstAgeGoodArm();
        armButtonMap = new FlywheelPDIFF();
        telemetry.addLine("Ready.");
        telemetry.update();
    }

    @Override
    public void loop() {
        driveButtonMap.loop(robot, this);
        armButtonMap.loop();
        robot.runLimelight(25);
        telemetry.update();
    }
}
