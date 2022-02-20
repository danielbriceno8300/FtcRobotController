package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.PI_RHO_BASE.PiRhoBaseAuto;
@Autonomous
public class AutoRedOutDepo extends PiRhoBaseAuto {
    @Override
    public void actions() {
        //drive out from the wall
        robot.drive(-6);
        sleep(50);

        //turn to the duckwheel redside
        robot.turn(90);
        sleep(50);

        //drives to duck wheel
        robot.drive(-16);
        sleep(250);

        //turn away from duck wheel and drive
        robot.turn(1);
        robot.drive(-20);
        sleep(50);

        //turn on intake for freight
        robot.setIntakePower(.75);
        sleep(50);

        //turn to hub and drive
        robot.turn(-85);
        robot.drive(-14);
        sleep(50);

        //drive to hub
        robot.drive(-5);
        sleep(50);
        //turn off intake
        robot.setIntakePower(0);

        //raise lift
        robot.setLiftPower(-.75);
        sleep(750);

        //deposits freight
        robot.bucket.setPosition(.6);
        sleep(1000);

        //drive away from hub
        robot.drive(20);

        //closes box servo
        robot.bucket.setPosition(.23);
        robot.setLiftPower(.75);
        sleep(750);
        robot.setLiftPower(0.0);

        //turn to duckwheel and drive
        robot.turn(0);
        robot.drive(29.5);
        sleep(50);

        //rotate duckwheel
        robot.turn(-2);
        //robot.drive(4);
        sleep(50);
        //turns wheel
        robot.setDuckWheel(-.60);
        sleep(1800);
        //turns duck wheel off
        robot.setDuckWheel(0);
        sleep(50);
        robot.setLiftPower(0.0);

        //park in storage thing
        robot.drive(-1);
        robot.turn(10);
        robot.drive(-9);
        sleep(1);
        robot.setLiftPower(0.0);





    }
}
