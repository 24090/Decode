package org.firstinspires.ftc.teamcode.subsystems.shooter

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
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
        val dash = FtcDashboard.getInstance()
        val p = TelemetryPacket()

        reads.update()
        p.put("leftVelocity", 0.0)
        p.put("rightVelocity", 0.0)
        p.put("targetVelocity", 1500.0)
        p.put("powerFeedforward", 0.0)
        dash.sendTelemetryPacket(p)
        intake.behaviour = Intake.IntakeBehaviour.Velocity(200.0)
        waitForStart()
        while (opModeIsActive()){
            reads.update()
            if (gamepad1.xWasPressed()) {
                targetVelocity += 50
            }
            if (gamepad1.yWasPressed()) {
                targetVelocity -= 50
            }
            intake.update()
            shooter.update()
            shooter.targetVelocity = targetVelocity
            val p = TelemetryPacket()
            intake.pusherLeft.position = if (gamepad1.x) pusherLeftForward else pusherLeftBack
            intake.pusherRight.position = if (gamepad1.x) pusherRightForward else pusherRightBack
            p.put("voltage", expansionHubVoltage)
            p.put("leftVelocity", shooter.motorLeft.velocity)
            p.put("rightVelocity", shooter.motorRight.velocity)
            p.put("powerFeedforward", shooter.velocityToPowerLUT.get(shooter.targetVelocity))
            dash.sendTelemetryPacket(p)
        }
    }
}