package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class PID {

    DcMotorEx motor;
    FtcDashboard dashboard;

    public PID(DcMotorEx motor, FtcDashboard dashboard) {
        this.motor = motor;
        this.dashboard = dashboard;
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    //Variables needed for measuring rate of change
    double lastTime = 0;
    double lastPosition = 0;
    double lastVelocity = 0;
    double integralErr = 0;

    // Setpoint in radians/second
    public void update(double setpoint, double kP, double kI, double kD) {

        //Time, position, change in time, change in position
        double currTime = System.currentTimeMillis();
        double position = motor.getCurrentPosition();
        double diffTime = currTime - lastTime;
        double diffPos = position - lastPosition;
        double lastError = 0;

        //Velocity (tick/s)
        double velocity = diffPos / diffTime;
        //Velocity (rotations/s)
        velocity *= (1.0 / 420.0);
        //Velocity (rad/s)
        velocity *= (2 * Math.PI);

        //Error is also in rad/s
        double error = setpoint - velocity;
        //Integral of the error (position)
        integralErr += error * diffTime; // I think this could also just be $position
        //Derivative of error, acceleration
        double derivativeErr = (error - lastError) / (currTime / lastTime);

        double output = (error * kP) + (integralErr * kI) + (derivativeErr * kD);
        motor.setPower(output);

        lastError = error;
        lastPosition = position;
        lastVelocity = velocity;
        lastTime = System.currentTimeMillis();
    }


}
