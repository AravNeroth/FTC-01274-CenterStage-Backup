package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.text.DecimalFormat;

public class Mecanum
{
    private DcMotorEx leftFront, leftRear, rightFront, rightRear;
    private double leftFrontPower, leftRearPower, rightFrontPower,rightRearPower, rotY, rotX, rx, x, y, denominator;
    private double offset = 1.1;

    DecimalFormat df = new DecimalFormat("#.##");
    // This rounds to two decimal places
    BNO055IMU imu;
    BNO055IMU.Parameters parameters;

    public Mecanum(HardwareMap hardwareMap)
    {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");

        leftRear.setDirection(DcMotorEx.Direction.REVERSE);
        leftFront.setDirection(DcMotorEx.Direction.REVERSE);

        imu = hardwareMap.get(BNO055IMU.class, "cIMU");
        // this is making a new object called 'parameters' that we use to hold the angle the imu is at
        parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
    }

    public void drive(Gamepad gamepad1)
    {
        y = gamepad1.left_stick_y;
        x = gamepad1.left_stick_x;
        rx = gamepad1.right_stick_x;

        double botHeading = -imu.getAngularOrientation().firstAngle;

        rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
        rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

        denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        leftFrontPower = (rotY + rotX + rx) / denominator;
        leftRearPower = (rotY - rotX + rx) / denominator;
        rightFrontPower = (rotY - rotX - rx) / denominator;
        rightRearPower = (rotY + rotX - rx) / denominator;
    }

    public void setMotorPower()
    {
        leftFront.setPower(leftFrontPower * offset);
        leftRear.setPower(leftRearPower * offset );
        rightFront.setPower(rightFrontPower * offset);
        rightRear.setPower(rightRearPower * offset );
    }
    public void setSlowDownMotorPower()
    {
        offset = 1.1 * 0.5;
    }

    public void setFullPower()
    {
        offset = 1.1;
    }

    public void resetIMU()
    {
        imu.initialize(parameters);
    }

//    public void setMotorPowerRounded()
//    {
//        leftFront.setPower(round(leftFrontPower));
//        leftRear.setPower(round(leftRearPower));
//        rightFront.setPower(round(rightFrontPower));
//        rightRear.setPower(round(rightRearPower));
//    }
}