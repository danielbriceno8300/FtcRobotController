package org.firstinspires.ftc.teamcode.PI_RHO_BASE;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class PiRhoHWMap {

    private static double TAU = Math.PI * 2;
    public DcMotorEx frontleft;
    public DcMotorEx backleft;
    public DcMotorEx frontright;
    public DcMotorEx backright;
    public DcMotorEx intake;
    public DcMotorEx lift;
    public DcMotorEx duckwheel;
    public DcMotorEx capArm;

    public Servo bucket;
    public CRServo intakeservo;
    public CRServo capServo;
    public CRServo bottomCapServo;


    private BNO055IMU imu;

    private double WHEEL_DIAMETER = 4.72441;
    //find wheel diameter on gobilda in mm to inches
    private double WHEEL_RADIUS = WHEEL_DIAMETER / 2;
    private double GEAR_RATIO = 1;
    // change to 20 instead of 13.7
    private double TICKS_PER_REV = 28 * 19.2;

    private double turnKp = 0.75;
    private double turnKi = 0.5;
    private double turnKd = 0.1;


    public PiRhoHWMap(HardwareMap hardwaremap) {
        frontleft = hardwaremap.get(DcMotorEx.class,"frontleft");
        backleft = hardwaremap.get(DcMotorEx.class,"backleft");
        frontright = hardwaremap.get(DcMotorEx.class,"frontright");
        backright = hardwaremap.get(DcMotorEx.class,"backright");
        duckwheel = hardwaremap.get(DcMotorEx.class, "duckwheel");
        lift = hardwaremap.get(DcMotorEx.class, "lift");
        imu = hardwaremap.get(BNO055IMU.class, "imu");
        bucket = hardwaremap.get(Servo.class, "bucketservo");
        intakeservo = hardwaremap.get(CRServo.class, "intakeservo");
        intake = hardwaremap.get(DcMotorEx.class, "intake");
        capArm = hardwaremap.get(DcMotorEx.class,"capArm");
        capServo = hardwaremap.get(CRServo.class, "capServo");
        bottomCapServo = hardwaremap.get(CRServo.class, "bottomCapServo");

        frontleft.setDirection(DcMotorEx.Direction.FORWARD);
        backleft.setDirection(DcMotorEx.Direction.FORWARD);
        frontright.setDirection(DcMotorEx.Direction.REVERSE);
        backright.setDirection(DcMotorEx.Direction.REVERSE);
        capArm.setDirection(DcMotorEx.Direction.REVERSE);

        frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        capArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
        PIRhoPID controller = new PIRhoPID(.69, 0, 0);
        PIRhoPID angleController = new PIRhoPID(turnKp, turnKi, turnKd);
        double initialAngle = imu.getAngularOrientation().firstAngle;
        double distanceTraveled = 0;
        double error = inches - distanceTraveled;
        double angleError = 0;
        while (Math.abs(error) > 1 || Math.abs(angleError) > Math.toRadians(3)){

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
            initialAngle = 0;
        }
        while(Math.abs(error) > Math.toRadians(2) || Math.abs(angleController.derivative) > Math.toRadians(15));
        robotrelative(0,0);
    }
    public double encoderTicksToInches(double ticks)
    {
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

    public void setIntakePower(double power){
        intake.setPower(power);
        intakeservo.setPower(power);

    }

    public void setLiftPower(double power){
        lift.setPower(power);

    }
    //sends power to the cap arm mechanism
    public void setCapArmMotor(double power)
    {
        capArm.setPower(power);
    }

    public void setCapServoPower(double power)
    {
        capServo.setPower(power);
    }

    public void setBottomCapServo(double power)
    {
        bottomCapServo.setPower(power);
    }

    public void setDuckWheel (double power) {
        duckwheel.setPower(power);

    }

}