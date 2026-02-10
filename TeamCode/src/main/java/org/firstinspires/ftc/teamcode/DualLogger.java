package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * DualLogger is used to make it able to log all telemetry shown on the phones
 * Logging can be used to check what problems the robot has encountered since it
 * records every telemetry shown
 */
public class DualLogger {

    Telemetry telemetry;

    public DualLogger(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public void addData(String caption, Object value) {
        telemetry.addData(caption, value);
        //FTC class RobotLog, used to write logs
        RobotLog.vv("7159RoboRavens", caption + ":" + value);
    }
}
