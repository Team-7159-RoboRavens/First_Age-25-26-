package org.firstinspires.ftc.teamcode.ButtonMaps;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.ComplexRobots.TrikeRobot;

public class SampleButtonMap extends TrikeAbstractButtonMap{
    //Instance Variables
    private double a;

    //If needed, you can pass some intital value that changes based on which teleop is used
    //to the constructor.
    public SampleButtonMap(double something){
        a = something;
    }

    @Override
    public void loop(TrikeRobot robot, OpMode opMode) {
        if(opMode.gamepad1.a) {
//            robot.armMotor.setPower(0);
        }
        //Other things would go here
    }
}
