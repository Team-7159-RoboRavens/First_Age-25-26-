package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="GoodTeleOpRed")
public class GoodTeleOpRed extends GoodTeleOpShared {
    @Override
    public void loop() {
        driveButtonMap.loop(robot, this);
        armButtonMap.loop(robot, this);
        robot.runLimelight(24);
        telemetry.update();
    }
}
