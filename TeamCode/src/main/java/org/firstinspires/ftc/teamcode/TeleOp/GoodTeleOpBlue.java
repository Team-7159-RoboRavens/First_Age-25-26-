package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "GoodTeleOpBlue")
public class GoodTeleOpBlue extends GoodTeleOpShared {
    @Override
    public void loop() {
        pinpoint.update();
        robot.runLimelight(20);
        driveButtonMap.loop(robot, this);
        armButtonMap.loop(robot, this);
    }
}
