package org.firstinspires.ftc.teamcode.opmodes.tests

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.commands.Forever
import org.firstinspires.ftc.teamcode.commands.Instant
import org.firstinspires.ftc.teamcode.commands.Parallel
import org.firstinspires.ftc.teamcode.commands.Race
import org.firstinspires.ftc.teamcode.commands.Sequence
import org.firstinspires.ftc.teamcode.commands.Sleep
import org.firstinspires.ftc.teamcode.commands.runBlocking
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake.Params.pusherWait
import org.firstinspires.ftc.teamcode.subsystems.reads.Reads
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter
import org.firstinspires.ftc.teamcode.util.timeSeconds

@TeleOp
@Config
class ShootCycleTesting: LinearOpMode() {
    companion object {
        @JvmField var velocity = 1400.0
        @JvmField var velocity2 = 1400.0

        @JvmField var hood = 0.0
        @JvmField var hood2 = 0.0


    }
    override fun runOpMode() {
        val telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        val intake = Intake(hardwareMap)
        val shooter = Shooter(hardwareMap)
        shooter.setHoodAngles(hood)
        val reads = Reads(hardwareMap)
        shooter.setTargetVelocities(velocity, velocity2)
        intake.behaviour = Intake.IntakeBehaviour.Grab
        val a = TelemetryPacket()
        telemetry.addData("left vel", shooter.targetVelocityLeft)
        telemetry.addData("right vel", shooter.targetVelocityRight)
        telemetry.addData("left vel real", shooter.motorLeft.velocity)
        telemetry.addData("right vel real", shooter.motorRight.velocity)
        telemetry.update()
        waitForStart()
        var startTime = 0.0
        runBlocking(Race(
            Forever {
                reads.update()
                intake.update()
                shooter.update()
                telemetry.addData("left vel real", shooter.motorLeft.velocity)
                telemetry.addData("right vel real", shooter.motorRight.velocity)
                telemetry.addData("left vel", shooter.targetVelocityLeft)
                telemetry.addData("right vel", shooter.targetVelocityRight)
                telemetry.update()
            },
            Sequence(
                shooter.waitForLeftVelocity(),
                Instant {
                    startTime = timeSeconds()
                },
                Parallel(
                    intake.releaseLeft(), // 0.05
                    intake.setAdjustThird()
                ),
                Sleep(pusherWait/2), // 0.025
                Parallel(
                    Instant {
                        intake.behaviour = Intake.IntakeBehaviour.TransferQuick
                        shooter.setHoodAngles(hood2)
                        shooter.setTargetVelocities(velocity2)
                    },
                    shooter.waitForVelocity(),
                    Sleep(0.05),
                ),
                intake.releaseDual(),
                Instant {
                    intake.behaviour = Intake.IntakeBehaviour.Grab
                    startTime = timeSeconds() - startTime
                },
            )
        ))
        telemetry.addData("time", startTime)
        telemetry.addData("left shot", shooter.shootCounterLeft)
        telemetry.addData("right shot", shooter.shootCounterRight)
        telemetry.update()
        while (opModeIsActive());
    }

}