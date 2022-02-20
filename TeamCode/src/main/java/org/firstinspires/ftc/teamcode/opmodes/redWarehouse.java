package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.PI_RHO_BASE.PiRhoBaseAuto;
@Autonomous
public class redWarehouse extends PiRhoBaseAuto {

    @Override
    public void actions() {
        //drive to hub
        robot.drive(-12);

        //raise lift
        robot.setLiftPower(-.75);
        sleep(750);

        //deposits freight
        robot.bucket.setPosition(.6);
        sleep(1000);

        //drive away from hub
        robot.drive(4);

        //closes box servo
        robot.bucket.setPosition(.23);
        robot.setLiftPower(.75);
        sleep(750);
        robot.setLiftPower(0.0);


        //back away from hub, turn and park
        robot.drive(-2);
        robot.turn(-89);
        robot.drive(-36);
    }
}
