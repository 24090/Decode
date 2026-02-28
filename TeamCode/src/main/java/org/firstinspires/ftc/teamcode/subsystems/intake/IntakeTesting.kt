package org.firstinspires.ftc.teamcode.subsystems.intake

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.commands.Forever
import org.firstinspires.ftc.teamcode.commands.Race
import org.firstinspires.ftc.teamcode.commands.Sequence
import org.firstinspires.ftc.teamcode.commands.runBlocking
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake.Params.runVelocity
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake.Params.runVelocityBack
import org.firstinspires.ftc.teamcode.subsystems.reads.Reads
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter

@TeleOp
@Config
class IntakeTesting(): LinearOpMode(){
    override fun runOpMode() {
        val intake = Intake(hardwareMap)
        val shooter = Shooter(hardwareMap)
        val reads = Reads(hardwareMap)
        val telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        val f: () -> Unit = {
            reads.update()
            intake.update()
            telemetry.addData("velocity", intake.motor.velocity)
            telemetry.addData("velocityBack", intake.motorBack.velocity)
            telemetry.addData("targetVelocity", runVelocity)
            telemetry.addData("back feedforward", Intake.backF * runVelocityBack)
            telemetry.addData("front feedforward", Intake.frontF * runVelocity)
            telemetry.addData("max", 2000)
            telemetry.addData("min", 0)
            telemetry.addData("avg", intake.stallTest.get())
            telemetry.addData("deriv", intake.stallTest.deriv())
            telemetry.update()
        }

        telemetry.addData("velocity", 0.0)
        telemetry.addData("velocityBack", 0.0)
        telemetry.addData("targetVelocity", 0.0)
        telemetry.addData("back feedforward", 0.0)
        telemetry.addData("front feedforward", 0.0)
        telemetry.update()
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