package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
        }
    }
    public void setIntake(){
        boolean spinPower = gamepad1.dpad_down;
        // intake wheel set power
        if (spinPower) {
            robot.setIntakePower(1);
        } else {
            robot.setIntakePower(0);
        }
    }

    public void setLift(){
        double liftPower = gamepad1.right_trigger - gamepad1.left_trigger;
        robot.setLiftPower(liftPower);
        double servoPosition = 0.15;
        if (gamepad1.triangle) servoPosition = 0.5;
        robot.bucket.setPosition(servoPosition);

    }

    public void setDuck(){
        double duckPower = 0;
        if (gamepad1.right_bumper) duckPower = .5;
        robot.setDuckWheel(duckPower);
    }
}
