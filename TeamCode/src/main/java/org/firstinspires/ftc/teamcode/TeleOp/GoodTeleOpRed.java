package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "GoodTeleOpRed")
public class GoodTeleOpRed extends GoodTeleOpShared {
    @Override
    public void loop() {
        pinpoint.update();
        robot.runLimelight(24);
        driveButtonMap.loop(robot, this);
        armButtonMap.loop(robot, this);
    }
}
