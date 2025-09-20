package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp()
public class TankDrive extends LinearOpMode {

    @Override
    public void runOpMode() {
        DcMotor left = hardwareMap.get(DcMotor.class, "left");
        DcMotor right = hardwareMap.get(DcMotor.class, "right");
        waitForStart();
        while (opModeIsActive()){
            left.setPower(-gamepad1.left_stick_y - gamepad1.left_stick_y);
            right.setPower(-gamepad1.left_stick_y + gamepad1.left_stick_x);
        }
    }
}
