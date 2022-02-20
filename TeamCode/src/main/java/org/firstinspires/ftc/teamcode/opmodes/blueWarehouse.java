package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.PI_RHO_BASE.PiRhoBaseAuto;
@Autonomous
public class blueWarehouse extends PiRhoBaseAuto {
    @Override
    public void actions() {
        //drive out from the wall
        robot.drive(-3);
        sleep(50);

        //turn to the right and drive
        robot.turn(-90);
        sleep(50);
        robot.drive(-8);

        //turn to hub
        robot.turn(-10);

        //turn on intake for freight
        robot.setIntakePower(.75);

        //drive to hub
        robot.drive(-9.5);
        sleep(50);
        //turn off intake
        robot.setIntakePower(0);

        //raise lift
        robot.setLiftPower(-.75);
        sleep(750);

        //deposits freight
        robot.bucket.setPosition(.6);
        sleep(1000);

        //drives into hub ever so slightly
        robot.drive(-1);
        sleep(50);

        //drive away from hub
        robot.drive(9);

        //closes box servo
        robot.bucket.setPosition(.23);
        robot.setLiftPower(.75);
        sleep(750);
        robot.setLiftPower(0.0);

        //turn to warehouse and park
        robot.turn(80);
        sleep(50);
        robot.drive(-60);



    }
}
