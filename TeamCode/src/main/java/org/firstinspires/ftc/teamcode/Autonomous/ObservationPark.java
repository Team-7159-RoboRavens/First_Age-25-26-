package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.ComplexRobots.FirstAgeTempbot;

@Autonomous()
public class ObservationPark extends LinearOpMode {
    public FirstAgeTempbot robot;
    public Encoder par;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new FirstAgeTempbot(hardwareMap, new Pose2d(new Vector2d(0,0),0), this);

        robot.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        par = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "leftBack")));
        int initialTick = par.getPositionAndVelocity().rawPosition;
        int currentTick = 0;
        telemetry.addData("init:", initialTick);
        telemetry.update();

        while(Math.abs(currentTick) < (robot.tiles(2)) && opModeIsActive()){
            robot.leftFront.setPower(0.3);
            robot.leftBack.setPower(0.3);
            robot.rightFront.setPower(0.3);
            robot.rightBack.setPower(0.3);
            currentTick = par.getPositionAndVelocity().rawPosition - initialTick;

            telemetry.addData("Encoder Value: ", currentTick);
            telemetry.update();
        }

//        Actions.runBlocking(
//                drive.driveBuilder()
//        );

         robot.leftFront.setPower(0);
         robot.leftBack.setPower(0);
         robot.rightFront.setPower(0);
         robot.rightBack.setPower(0);

         telemetry.update();


    }

//    public class ExtendSlides implements Action {
//        private boolean initalized = false;
//
//        public boolean run(TelemetryPacket packet) {
//            double vel = motor.getVelocity();
//            packet.put("shooterVelocity", vel);
//            return vel < 10_000.0;
//        }
//
//    }
}
