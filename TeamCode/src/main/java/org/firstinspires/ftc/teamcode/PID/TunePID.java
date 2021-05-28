package org.firstinspires.ftc.teamcode.PID;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
public class TunePID extends OpMode {

    DcMotorEx motor;
    String motorName = "frontLeft";
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashTelemetry = dashboard.getTelemetry();

    PID pid;

    //Rad/s
    double setpoint = 50;
    double kP = 0.1;
    double kI = 0;
    double kD = 0;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, motorName);
        pid = new PID(motor, dashTelemetry);
    }

    @Override
    public void loop() {
        pid.update(setpoint, kP, kI, kD);
        pid.publishTelemetry(true);
    }
}
