package org.firstinspires.ftc.teamcode.ButtonMaps.Arm;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.ButtonMaps.MotorPowers;
import org.firstinspires.ftc.teamcode.ButtonMaps.NoWheelsAbstractButtonMap;
import org.firstinspires.ftc.teamcode.ButtonMaps.ServoAbstractButtonMapGood;
import org.firstinspires.ftc.teamcode.ComplexRobots.ServoGoodBot;
import org.firstinspires.ftc.teamcode.ComplexRobots.ShootOnlyBot;
import org.firstinspires.ftc.teamcode.limelightData;

public class buttonTester extends NoWheelsAbstractButtonMap {
    //TODO: Change back to private final when done with dash
    private MotorPowers mp;// = new MotorPowers(0);
    private double servoPosition;
    private double stage = 0;
    private double timeSince;
    private double timeBuffer = 3000;

    //These magic numbers are not final and should be iteratively tested.
    public static double baseShotPower = .40;
    public static double limelightPowerMultiplier = 1.18;
    public static double limelightBaseDistance = 100;
    public static double nonLinearPower = 1.0028;
    public static double shootVel;
    public static double targetVel;
    static double joystickDeadZone = .15;



    @Override
    public void loop(ShootOnlyBot robot, OpMode opMode) {
        if (opMode.gamepad2.y) {
            robot.intakeMotor1.setPower(-1);
            opMode.telemetry.addLine("Intake1");
        }

        if (opMode.gamepad2.x) {
            robot.intakeMotor2.setPower(-1);
            opMode.telemetry.addLine("Intake2");
        }
    }
    public static double velocityShot(double x) {
        return (2.07096 * Math.pow(10, -16) * .3 * Math.pow(x, 2) + 7.81571 * x + 550.14286);
    }
}
