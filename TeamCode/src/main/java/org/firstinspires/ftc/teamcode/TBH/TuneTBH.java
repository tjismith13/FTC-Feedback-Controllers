package org.firstinspires.ftc.teamcode.TBH;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PID.PID;

@TeleOp
public class TuneTBH extends OpMode {

    DcMotorEx motor;
    String motorName = "frontLeft";
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashTelemetry = dashboard.getTelemetry();

    TBH tbh;

    //Rad/s
    double setpoint = 50;
    double k = 0.1;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, motorName);
        tbh = new TBH(motor, dashTelemetry);
    }

    @Override
    public void loop() {
        tbh.update(setpoint, k);
        tbh.publishTelemetry(true);
    }
}
