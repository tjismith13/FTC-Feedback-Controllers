package org.firstinspires.ftc.teamcode.BB;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PID.PID;

@TeleOp
public class TuneBB extends OpMode {

    DcMotorEx motor;
    String motorName = "frontLeft";
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashTelemetry = dashboard.getTelemetry();

    BB bb;

    //Rad/s
    double setpoint = 30;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, motorName);
        bb = new BB(motor, dashTelemetry);
    }

    @Override
    public void loop() {
        bb.update(setpoint);
        bb.publishTelemetry(true);
    }
}
