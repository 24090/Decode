package org.firstinspires.ftc.teamcode.subsystems.intake

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.commands.Forever
import org.firstinspires.ftc.teamcode.commands.Race
import org.firstinspires.ftc.teamcode.commands.Sequence
import org.firstinspires.ftc.teamcode.commands.runBlocking
import org.firstinspires.ftc.teamcode.subsystems.reads.Reads
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter

@TeleOp
@Config
class IntakeTesting(): LinearOpMode(){
    companion object {
        @JvmField var targetVelocity = 0.0
    }
    override fun runOpMode() {
        val intake = Intake(hardwareMap)
        val shooter = Shooter(hardwareMap)
        val reads = Reads(hardwareMap)
        val dash = FtcDashboard.getInstance()
        val p = TelemetryPacket()
        val f = {
            reads.update()
            intake.update()
            intake.behaviour.target = targetVelocity
            val p = TelemetryPacket()
            p.put("velocity", intake.motor.velocity)
            p.put("targetVelocity", intake.behaviour.target)
            p.put("powerFeedforward", Intake.kF * intake.behaviour.target)
            dash.sendTelemetryPacket(p)
        }
        p.put("velocity", 0.0)
        p.put("targetVelocity", 1500.0)
        p.put("powerFeedforward", 0.0)
        dash.sendTelemetryPacket(p)
        waitForStart()
        shooter.motorLeft.power = 0.12
        shooter.motorRight.power = 0.12
        while (opModeIsActive()){
            reads.update()

            if (gamepad1.leftBumperWasPressed()){
                runBlocking(Race(
                    Forever(f),
                    intake.releaseLeft()
                ))
            }
            if (gamepad1.rightBumperWasPressed()){
                runBlocking(Race(
                    Forever(f),
                    intake.releaseRight()
                ))
            }
            if (gamepad1.xWasPressed()){
                runBlocking(Race(
                    Forever(intake::update),
                    Sequence(
                        intake.fullAdjustThird(),
                        intake.stop()
                    )
                ))
            }
            f()

        }
    }

}