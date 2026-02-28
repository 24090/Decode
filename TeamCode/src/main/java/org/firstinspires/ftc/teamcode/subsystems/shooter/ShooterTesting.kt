package org.firstinspires.ftc.teamcode.subsystems.shooter

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake.Params.pusherLeftBack
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake.Params.pusherLeftForward
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake.Params.pusherRightBack
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake.Params.pusherRightForward
import org.firstinspires.ftc.teamcode.subsystems.reads.Reads
import org.firstinspires.ftc.teamcode.subsystems.reads.VoltageReader.expansionHubVoltage


@TeleOp
@Config
class ShooterTesting(): LinearOpMode(){
    companion object {
        @JvmField var targetVelocity = 1500.0
    }
    override fun runOpMode() {
        val reads = Reads(hardwareMap)
        val shooter = Shooter(hardwareMap)
        val intake = Intake(hardwareMap)
        val telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        reads.update()
        telemetry.addData("leftVelocity", 0.0)
        telemetry.addData("rightVelocity", 0.0)
        telemetry.addData("targetVelocity", 1500.0)
        telemetry.addData("powerFeedforward", 0.0)
        intake.behaviour = Intake.IntakeBehaviour.Grab
        waitForStart()
        while (opModeIsActive()){
            reads.update()
            if (gamepad1.aWasPressed()) {
                targetVelocity += 50
            }
            if (gamepad1.bWasPressed()) {
                targetVelocity -= 50
            }
            intake.update()
            shooter.update()
            shooter.setTargetVelocities(targetVelocity)
            val p = TelemetryPacket()
            intake.pusherLeft.position = if (gamepad1.x) pusherLeftForward else pusherLeftBack
            intake.pusherRight.position = if (gamepad1.x) pusherRightForward else pusherRightBack
            telemetry.addData("voltage", expansionHubVoltage)
            telemetry.addData("leftVelocity", shooter.motorLeft.velocity)
            telemetry.addData("rightVelocity", shooter.motorRight.velocity)
            telemetry.addData("powerFeedforwardLeft", shooter.velocityToPowerLUT.get(shooter.targetVelocityLeft))
        }
    }
}