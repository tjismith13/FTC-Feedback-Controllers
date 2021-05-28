package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class BB {

    DcMotorEx motor;
    FtcDashboard dashboard;

    public BB(DcMotorEx motor, FtcDashboard dashboard) {
        this.motor = motor;
        this.dashboard = dashboard;
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    //Variables for calculating velocity
    double velocity;
    double lastTime = 0;
    double lastPosition = 0;

    //Original sign of the error
    double sign;
    boolean started = false;
    public void update(double setpoint) {

        //Original sign of error used to ensure we don't flip between 1 and -1, just +/-1 and 0
        if(!started) {
            started = true;
            if(setpoint >= 0) sign = 1;
            else sign = -1;
        }
        double position = motor.getCurrentPosition();
        double time = System.currentTimeMillis();
        double diffTime = time - lastTime;
        double diffPos = position - lastPosition;

        //Velocity (tick/s)
        velocity = diffPos / diffTime;
        //Velocity (rotations/s)
        velocity *= (1.0 / 420.0);
        //Velocity (rad/s)
        velocity *= (2 * Math.PI);

        int output = 0;
        if(velocity <= setpoint && sign == 1) output = 1;
        else if(velocity >= setpoint && sign == 1) output = 0;
        else if(velocity < setpoint && sign == -1) output = 0;
        else output = -1;

        motor.setPower(output);

        lastPosition = position;
        lastTime = System.currentTimeMillis();
    }

    double getVelocity() {
        return velocity;
    }
}
