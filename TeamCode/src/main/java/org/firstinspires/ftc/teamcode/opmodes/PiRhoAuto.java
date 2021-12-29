package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.PI_RHO_BASE.PiRhoBaseAuto;

@Autonomous
public class PiRhoAuto extends PiRhoBaseAuto {
    @Override
    public void actions() {
        robot.turn(90);
        robot.drive(48);
    }
}