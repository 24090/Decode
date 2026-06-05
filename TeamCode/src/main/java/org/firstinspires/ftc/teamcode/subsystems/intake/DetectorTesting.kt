package org.firstinspires.ftc.teamcode.subsystems.intake

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@TeleOp
class DetectorTesting: LinearOpMode() {
    override fun runOpMode() {
        val intake = Intake(hardwareMap)
        val telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        waitForStart()
        while (opModeIsActive()){
            telemetry.addData("detectorLeft", intake.detectorLeft.state)
            telemetry.addData("detectorRight", intake.detectorRight.state)
            telemetry.update()
        }
    }
}