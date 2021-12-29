package org.firstinspires.ftc.teamcode.PI_RHO_BASE;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PIRhoPID {
    double Kp; double Ki; double Kd;
    ElapsedTime timer = new ElapsedTime();
    protected double previousError = 0;
    protected double integral_sum = 0;
    protected double derivative = 0;
    public PIRhoPID (double Kp, double Ki, double Kd){
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        timer.reset();
    }

    public double calculate(double error){
        double p = error * Kp;
        integral_sum += error * timer.seconds();
        derivative = (error - previousError)/timer.seconds();
        double d = Kd * derivative;
        double i = integral_sum * Ki;
        double output = p + i + d;
        // reset timer for the delta time between loops
        timer.reset();
        return output;
    }

    public double getDerivative() {
        return derivative;
    }
}
