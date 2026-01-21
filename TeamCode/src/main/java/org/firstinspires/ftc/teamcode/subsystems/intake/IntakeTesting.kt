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
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake.Params.runVelocity
import org.firstinspires.ftc.teamcode.subsystems.reads.Reads
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter

@TeleOp
@Config
class IntakeTesting(): LinearOpMode(){
    override fun runOpMode() {
        val intake = Intake(hardwareMap)
        val shooter = Shooter(hardwareMap)
        val reads = Reads(hardwareMap)
        val dash = FtcDashboard.getInstance()
        val p = TelemetryPacket()
        val f = {
            reads.update()
            intake.update()
            val p = TelemetryPacket()
            p.put("velocity", intake.motor.velocity)
            p.put("velocityBack", intake.motorBack.velocity)
            p.put("targetVelocity", runVelocity)
            p.put("back feedforward", Intake.backF * runVelocity)
            p.put("front feedforward", Intake.frontF * runVelocity)
            dash.sendTelemetryPacket(p)
        }

        p.put("velocity", 0.0)
        p.put("velocityBack", 0.0)
        p.put("targetVelocity", 0.0)
        p.put("back feedforward", 0.0)
        p.put("front feedforward", 0.0)
        dash.sendTelemetryPacket(p)
        waitForStart()
        intake.behaviour = Intake.IntakeBehaviour.Grab
        shooter.motorLeft.power = 0.12
        shooter.motorRight.power = 0.12
        while (opModeIsActive()){
            reads.update()

            shooter.motorLeft.power = 0.12
            shooter.motorRight.power = 0.12

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
                    Forever{
                        reads::update::invoke::invoke.invoke()
                        intake::update::invoke::invoke::invoke::invoke.invoke()
                    },
                    Sequence(
                        intake.fullAdjustThird(),
                        intake.spinUp()
                    )
                ))
            }
            f()

        }
    }

}