package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="GoodTeleOpBlue")
public class GoodTeleOpBlue extends GoodTeleOpShared {
    @Override
    public void loop() {
        driveButtonMap.loop(robot, this);
        armButtonMap.loop(robot, this);
        robot.runLimelight(20);
        telemetry.update();
//        pinpoint.update();
    }
}
