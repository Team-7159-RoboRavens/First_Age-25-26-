package org.firstinspires.ftc.teamcode.ButtonMaps.Arm;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.ButtonMaps.MotorPowers;
import org.firstinspires.ftc.teamcode.ButtonMaps.NoWheelsAbstractButtonMap;
import org.firstinspires.ftc.teamcode.ButtonMaps.ServoAbstractButtonMapGood;
import org.firstinspires.ftc.teamcode.ComplexRobots.ServoGoodBot;
import org.firstinspires.ftc.teamcode.ComplexRobots.ShootOnlyBot;
import org.firstinspires.ftc.teamcode.limelightData;

public class GoodArmNoWheels extends NoWheelsAbstractButtonMap {
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
    static double joystickDeadZone = .1;



    @Override
    public void loop(ShootOnlyBot robot, OpMode opMode) {
        if (opMode.gamepad1.dpad_up) {
            targetVel = velocityShot(280);
            opMode.telemetry.addData("Target Velocity", targetVel);

            robot.ShootMotor.setPower((targetVel - robot.ShootMotor.getVelocity()) / 120);

            robot.ShootMotor2.setPower((targetVel - robot.ShootMotor2.getVelocity()) / 120);
        }
        else {
            robot.ShootMotor.setVelocity(0);
            robot.ShootMotor2.setVelocity(0);
        }
        opMode.telemetry.addData("Shoot 1 Velocity", robot.ShootMotor.getVelocity());
        opMode.telemetry.addData("Shoot 2 Velocity", robot.ShootMotor2.getVelocity());

    }
    public static double velocityShot(double x) {
        return 2.84926 * x + 1233.65423;
    }
}
