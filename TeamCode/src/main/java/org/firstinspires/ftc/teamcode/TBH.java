package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class TBH {

    DcMotorEx motor;
    FtcDashboard dashboard;

    public TBH(DcMotorEx motor, FtcDashboard dashboard) {
        this.motor = motor;
        this.dashboard = dashboard;
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    //Variables for rate of change
    double lastTime = 0;
    double lastPosition = 0;
    double integralErr = 0;

    double lastErrorSign = 1;
    double takeBackFactor = 1;

    //output (for getter)
    double velocity;
    public void update(double setpoint, double gain) {

        //Time, position, change in time, change in position
        double currTime = System.currentTimeMillis();
        double position = motor.getCurrentPosition();
        double diffTime = currTime - lastTime;
        double diffPos = position - lastPosition;

        //Velocity (tick/s)
        velocity = diffPos / diffTime;
        //Velocity (rotations/s)
        velocity *= (1.0 / 420.0);
        //Velocity (rad/s)
        velocity *= (2 * Math.PI);

        //Error is also in rad/s
        double error = setpoint - velocity;

        //Check for crossover
        double sign;
        if(error >= 0) sign = 1;
        else sign = -1;

        if(sign != lastErrorSign) takeBackFactor *= 2;


        //Integral of the error (position)
        integralErr += error * diffTime; // I think this could also just be $position

        double output = (gain * integralErr) / takeBackFactor;
        motor.setPower(output);

        lastTime = System.currentTimeMillis();
        lastPosition = position;
    }

    double getVelocity() {return velocity;}
}
