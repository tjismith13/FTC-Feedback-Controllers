package org.firstinspires.ftc.teamcode.PID;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
public class RunPID extends OpMode {

    DcMotorEx motor;
    String motorName = "frontLeft";
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashTelemetry = dashboard.getTelemetry();

    //Rad/s
    double setpoint = 50;
    double kP = 0.1;
    double kI = 0;
    double kD = 0;

    PID pid;

    //For calculating rise time and settling time
    double startTime;
    double tenPTime;
    double ninetyPTime;

    double tenPercent = setpoint * 0.1;
    double ninetyPercent = setpoint * 0.9;

    boolean surpassedTen = false;
    boolean surpassedNinety = false;

    double plus1 = setpoint *= 1.01;
    double minus1 = setpoint *= 0.99;

    boolean inRangeLast = false;
    boolean done = false;

    double settlingTimeClock;
    double settlingTime;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, motorName);
        pid = new PID(motor, dashTelemetry);

        startTime = System.currentTimeMillis();
    }

    @Override
    public void loop() {
        pid.update(setpoint, kP, kI, kD);

        if(pid.getVelocity() > tenPercent && !surpassedTen) {
            tenPTime = System.currentTimeMillis();
            surpassedTen = true;
        }
        if(pid.getVelocity() > ninetyPercent && !surpassedNinety) {
            ninetyPTime = System.currentTimeMillis();
            surpassedNinety = true;
        }
        if(surpassedNinety && surpassedTen) {
            dashTelemetry.addData("Rise time", (ninetyPTime - tenPTime) / 1000);
        }

        if(setpoint < plus1 && setpoint > minus1) {
            inRangeLast = true;
            if(!inRangeLast) {
                settlingTimeClock = System.currentTimeMillis();
            }
            else {
                if(settlingTimeClock > 3000) {
                    done = true;
                    settlingTime = System.currentTimeMillis() - startTime;
                }
            }
        }
        else inRangeLast = false;
        if(done) dashTelemetry.addData("settling Time", settlingTime);
        pid.publishTelemetry(false);
        dashTelemetry.update();
    }
}
