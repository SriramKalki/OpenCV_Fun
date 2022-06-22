package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {
    private double Kp, Ki, Kd;
    private double targetAngle;
    private ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;
    private double accumulatedError = 0;
    private double lastTime = -1;
    private double lastSlope = 0;



    public PIDController(double Kp, double Ki, double Kd, double targetAngle){
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.targetAngle = targetAngle;
    }


    public double update(double currentAngle){
        //P
        double error = targetAngle - currentAngle;
        error %= 360;
        error += 360;
        error %= 360;
        if (error > 180) {
            error -= 360;
        }

        //I
        accumulatedError *= Math.signum(error);
        accumulatedError += error;
        if (Math.abs(error) < 2) {
            accumulatedError = 0;
        }

        //D
        double slope = 0;
        if (lastTime > 0) {
            slope = (error - lastError) / (timer.milliseconds() - lastTime);
        }
        lastSlope = slope;
        lastError = error;
        lastTime = timer.milliseconds();

        double motorPower = 0.1 * Math.signum(error)
                + 0.9 * Math.tanh(Kp * error + Ki * accumulatedError - Kd * slope);
        return motorPower;



    }





}
