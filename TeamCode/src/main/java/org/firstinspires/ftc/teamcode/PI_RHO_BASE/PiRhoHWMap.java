package org.firstinspires.ftc.teamcode.PI_RHO_BASE;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class PiRhoHWMap {

    private static double TAU = Math.PI * 2;
    public DcMotorEx frontleft;
    public DcMotorEx backleft;
    public DcMotorEx frontright;
    public DcMotorEx backright;
    private BNO055IMU imu;

    private double WHEEL_DIAMETER = 3.7795276;
    //find wheel diameter on gobilda in mm to inches
    private double WHEEL_RADIUS = WHEEL_DIAMETER / 2;
    private double GEAR_RATIO = 1;
    // change to 20 instead of 13.7
    private double TICKS_PER_REV = 28 * 13.7;

    private double turnKp = 0.5;
    private double turnKi = 0;
    private double turnKd = 0;


    public PiRhoHWMap(HardwareMap hardwaremap) {
        frontleft = hardwaremap.get(DcMotorEx.class,"FrontLeft");
        backleft = hardwaremap.get(DcMotorEx.class,"BackLeft");
        frontright = hardwaremap.get(DcMotorEx.class,"FrontRight");
        backright = hardwaremap.get(DcMotorEx.class,"BackRight");
        frontleft.setDirection(DcMotorEx.Direction.FORWARD);
        backleft.setDirection(DcMotorEx.Direction.FORWARD);
        frontright.setDirection(DcMotorEx.Direction.REVERSE);
        backright.setDirection(DcMotorEx.Direction.REVERSE);
        frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        imu = hardwaremap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);


    }
    private void setdrivepower(double left, double right){
        frontleft.setPower(left);
        backleft.setPower(left);
        frontright.setPower(right);
        backright.setPower(right);
    }
    public void robotrelative(double x, double turn){
        double left = x + turn;
        double right = x - turn;
        setdrivepower(left, right);
    }
    private void resetencoder(){
        frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void drive(double inches){
        resetencoder();
        PIRhoPID controller = new PIRhoPID(.09, 0, 0);
        PIRhoPID angleController = new PIRhoPID(turnKp, turnKi, turnKd);
        double initialAngle = imu.getAngularOrientation().firstAngle;
        double distanceTraveled = 0;
        double error = inches - distanceTraveled;
        double angleError = 0;
        while (Math.abs(error) > 1 || encoderTicksToInches(Math.abs(frontleft.getVelocity())) > 1 || Math.abs(angleError) > Math.toRadians(3)){

            distanceTraveled = encoderTicksToInches((double)frontleft.getCurrentPosition()
                    + (double)frontright.getCurrentPosition()) / 2;
            error = inches - distanceTraveled;

            double angle = normalizeAngleRR(imu.getAngularOrientation().firstAngle - initialAngle);

            angleError = normalizeAngle(0 - angle);

            robotrelative(controller.calculate(error), angleController.calculate(angleError));
        }
        robotrelative(0,0);
    }

    public void turn(double degrees){
        PIRhoPID angleController = new PIRhoPID(turnKp, turnKi, turnKd);
        double radians = Math.toRadians(degrees);
        double initialAngle = imu.getAngularOrientation().firstAngle;
        double error;
        do {
            double currentAngle = normalizeAngleRR(imu.getAngularOrientation().firstAngle + initialAngle);
            error = normalizeAngle(radians - currentAngle);
            double power = angleController.calculate(error);
            robotrelative(0, power);
            System.out.println("error is " + error + " derivative is " + angleController.derivative + " 3 degrees is " + Math.toRadians(3));
        } while(Math.abs(error) > Math.toRadians(2) || Math.abs(angleController.derivative) > 15);
        robotrelative(0,0);
    }
    public double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public static double normalizeAngleRR(double radians) {
        double modifiedAngle = radians % TAU;
        modifiedAngle = (modifiedAngle + TAU) % TAU;
        return modifiedAngle;
    }

    public double normalizeAngle(double radians) {
        return AngleWrap(-normalizeAngleRR(radians));
    }

    public double AngleWrap(double angle){
        while (angle<-Math.PI){
            angle += 2.0* Math.PI;
        }
        while (angle> Math.PI){
            angle -= 2.0* Math.PI;
        }
        return angle;
    }

}