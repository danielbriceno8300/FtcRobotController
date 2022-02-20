package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.PI_RHO_BASE.PiRhoHWMap;

@TeleOp
public class PiRhoTeleOp extends LinearOpMode {

    public PiRhoHWMap robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new PiRhoHWMap(hardwareMap);
        waitForStart();
        while (opModeIsActive()){
            robot.robotrelative(-gamepad1.left_stick_y,gamepad1.right_stick_x);
            setIntake();
            setLift();
            setDuck();
            reverseIntake();
            setCap();
            capServo();
            bottomCapServo();
        }
    }

    public void reverseIntake(){
        boolean reverseIntake = gamepad1.x;
        if (reverseIntake){
            robot.setIntakePower(-1);
        }
    }

    public void setIntake(){
        boolean spinPower = gamepad1.left_bumper;
        // intake wheel set power
        if (spinPower) {
            robot.setIntakePower(1);
        } else
            {
            robot.setIntakePower(0);
        }
    }

    public void setLift()
    {
        double liftPower = gamepad1.left_trigger - gamepad1.right_trigger;
        robot.setLiftPower(liftPower);
        double position = robot.lift.getCurrentPosition();
        telemetry.addData("lift position: ", position);
        telemetry.update();
        if (position <= -1550 || position >= -100)
        {
            robot.setLiftPower(0);
            telemetry.addData("goal reached: ", position);
        }
        double servoPosition = 0.23;
        if (gamepad1.triangle) servoPosition = 0.6;
        robot.bucket.setPosition(servoPosition);


    }

    public void setDuck()
    {
        double duckPower = 0;
        if (gamepad1.right_bumper) duckPower = .63;
        robot.setDuckWheel(duckPower);
    }

    public void capServo()
    {
        double capPower = 0;
        //arm goes out
        if (gamepad2.dpad_right) capPower = 1;
        robot.setCapArmMotor(capPower);

        //arm goes in
        if (gamepad2.dpad_left) capPower = -1;
        robot.setCapArmMotor(capPower);
    }

    public void setCap()
    {
        double servoPower = 0;
        //arm goes up
        if (gamepad2.dpad_up) servoPower = 1;
        robot.setCapServoPower(servoPower);

        //arm goes down
        if (gamepad2.dpad_down) servoPower = -1;
        robot.setCapServoPower(servoPower);
    }

    //bottom most servo for the monstrosity we made the night and morning of the state championship
    public void bottomCapServo()
    {
        double bottomservo = 0;
        if (gamepad2.right_bumper) bottomservo = 1;
        robot.setBottomCapServo(bottomservo);
        if (gamepad2.left_bumper) bottomservo = -1;
        robot.setBottomCapServo(bottomservo);

    }



}
