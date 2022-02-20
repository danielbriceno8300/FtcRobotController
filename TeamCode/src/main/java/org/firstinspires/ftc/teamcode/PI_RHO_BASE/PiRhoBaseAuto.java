package org.firstinspires.ftc.teamcode.PI_RHO_BASE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class PiRhoBaseAuto extends LinearOpMode {
    public PiRhoHWMap robot;
    public abstract void actions();
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new PiRhoHWMap(hardwareMap);
        telemetry.addData("Ready for start","Yes");
        telemetry.update();
        waitForStart();
        actions();
    }

}
