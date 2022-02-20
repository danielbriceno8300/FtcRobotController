package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.PI_RHO_BASE.PiRhoBaseAuto;
@Autonomous
public class duckAutoBlueOutDepo extends PiRhoBaseAuto {
    @Override
    public void actions() {
        //drive out from the wall
        robot.drive(-5);
        sleep(500);
        //turn to the right
        robot.turn(90);
        sleep(500);
        //drives to duck wheel
        robot.drive(13.5);
        sleep(1000);
        //turn on intake
        robot.setIntakePower(1);
        //turns wheel
        robot.setDuckWheel(.60);
        sleep(1600);
        //turns duck wheel off
        robot.setDuckWheel(0);
        //turn towards hub
        robot.turn(1);
        sleep(1000);
        //drives towards hub
        robot.drive(-22);
        //turn towards warehouse
        robot.turn(90);
        //drive to hub
        robot.drive(-21.5);
        sleep(1000);
        //turns off intake
        robot.setIntakePower(0);
        //raise lift
        robot.setLiftPower(-.75);
        sleep(750);
        //turns off intake
        robot.setIntakePower(0);
        sleep(500);
        //deposits freight
        robot.bucket.setPosition(.6);
        //drives into hub ever so slightly
        robot.drive(-.5);
        sleep(1000);
        //drives towards parking zone
        robot.drive(25);
        //closes box servo
        robot.bucket.setPosition(.23);
        robot.setLiftPower(.75);
        sleep(750);
        robot.setLiftPower(0.0);
        //turn to duck wheel for second time
        robot.turn(0);
        sleep(1000);
        //drives into parking zone
        robot.drive(6);

    }
}
