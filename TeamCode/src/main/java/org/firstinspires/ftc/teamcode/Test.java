package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
public class Test extends OpMode {

    DcMotorEx motor;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashTelemetry = dashboard.getTelemetry();

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, "frontLeft");
    }

    double lastTime = 0;
    double lastPosition = 0;
    @Override
    public void loop() {

        motor.setPower(0.95);
        double position = motor.getCurrentPosition();
        double time = System.currentTimeMillis() / 1000.0;

        //Velocity (ticks/ s)
        double velocity = (position - lastPosition) / (time - lastTime);
        //Velocity (rotations/s)
        velocity *= (1.0 / 420.0);
        //Velocity (rad/s)
        velocity *= (2 * Math.PI);

        dashTelemetry.addData("Velocity", velocity);
        dashTelemetry.update();

        lastTime = time;
        lastPosition = position;
    }
}
