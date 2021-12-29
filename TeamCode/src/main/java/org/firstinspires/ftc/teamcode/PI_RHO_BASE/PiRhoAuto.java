package org.firstinspires.ftc.teamcode.PI_RHO_BASE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
@Autonomous
public class PiRhoAuto extends PiRhoBaseAuto {

    @Override
    public void actions() {
        robot.turn(90);
        robot.drive(48);
    }
}