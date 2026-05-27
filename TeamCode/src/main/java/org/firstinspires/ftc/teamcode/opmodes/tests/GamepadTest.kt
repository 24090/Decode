package org.firstinspires.ftc.teamcode.opmodes.tests

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.commands.Forever
import org.firstinspires.ftc.teamcode.commands.Race
import org.firstinspires.ftc.teamcode.commands.runBlocking
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake
import org.firstinspires.ftc.teamcode.subsystems.reads.Reads
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter

@TeleOp()
class GamepadTest: LinearOpMode(){
    override fun runOpMode() {
        val telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        waitForStart()
        while (opModeIsActive()){
            telemetry.addData("gamepad1.left_x", gamepad1.left_stick_x)
            telemetry.update()
        }
    }
}