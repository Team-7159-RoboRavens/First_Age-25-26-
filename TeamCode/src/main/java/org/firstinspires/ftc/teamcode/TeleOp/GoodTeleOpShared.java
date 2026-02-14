package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorGoBildaPinpoint;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.ButtonMaps.Arm.FirstAgeGoodArm;
import org.firstinspires.ftc.teamcode.ButtonMaps.Drive.LiamPolarDrive;
import org.firstinspires.ftc.teamcode.ButtonMaps.Drive.LiamPolarDriveGood;
import org.firstinspires.ftc.teamcode.ButtonMaps.ServoAbstractButtonMap;
import org.firstinspires.ftc.teamcode.ButtonMaps.ServoAbstractButtonMapGood;
import org.firstinspires.ftc.teamcode.ComplexRobots.ServoGoodBot;
import org.firstinspires.ftc.teamcode.ComplexRobots.ServoTempBot;
import org.firstinspires.ftc.teamcode.DualLogger;

public abstract class GoodTeleOpShared extends OpMode {
    //Global Variables
    ServoGoodBot robot;

    //Button Maps
    ServoAbstractButtonMapGood driveButtonMap;
    ServoAbstractButtonMapGood armButtonMap;
    GoBildaPinpointDriver pinpoint;

    @Override
    public void init() {
        DualLogger dualLogger;
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        dualLogger = new DualLogger(telemetry);

        // Set the location of the robot - this should be the place you are starting the robot from
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS, 0));
        dualLogger.addLine("Initializing, please wait...");
        telemetry.update();
        pinpoint.resetPosAndIMU();
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.setOffsets(2.5, -7.5, DistanceUnit.INCH);
        robot = new ServoGoodBot(hardwareMap, new Pose2d(0, 0, 0), this, dualLogger);
        robot.setPinpoint(pinpoint);
        driveButtonMap = new LiamPolarDriveGood();
        armButtonMap = new FirstAgeGoodArm();
        dualLogger.addLine("Ready.");

        // Configure the sensor

    }
}
