package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class PID {

    DcMotorEx motor;
    FtcDashboard dashboard;

    public PID(DcMotorEx motor, FtcDashboard dashboard) {
        this.motor = motor;
        this.dashboard = dashboard;
    }

    double lastTime = 0;

    public void update(double setpoint) {
        double velocity = motor.getVelocity(AngleUnit.RADIANS);
        double currTime = System.currentTimeMillis();

    }


}
