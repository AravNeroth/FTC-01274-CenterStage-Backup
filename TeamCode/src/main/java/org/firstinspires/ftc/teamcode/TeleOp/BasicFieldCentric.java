package org.firstinspires.ftc.teamcode.TeleOp;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Mecanum;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp

public class BasicFieldCentric extends OpMode {
    private ElapsedTime runTime;
    private Mecanum driveTrain;

    @Override
    public void init() {
        driveTrain = new Mecanum(hardwareMap);
    }

    @Override
    public void loop() {

        driveTrain.drive(gamepad1);
        driveTrain.setMotorPower();

    }
}