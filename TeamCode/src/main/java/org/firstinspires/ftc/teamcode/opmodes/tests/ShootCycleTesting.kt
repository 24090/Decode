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

@TeleOp
@Config
class ShootCycleTesting: LinearOpMode() {
    companion object {
        @JvmField var targetValue = 1400.0
    }
    override fun runOpMode() {
        val telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        val intake = Intake(hardwareMap)
        val shooter = Shooter(hardwareMap)
        val reads = Reads(hardwareMap)
        shooter.setTargetVelocities(targetValue)
        intake.behaviour = Intake.IntakeBehaviour.Grab
        val a = TelemetryPacket()
        telemetry.addData("left shot", shooter.shootCounterLeft)
        telemetry.addData("right shot", shooter.shootCounterRight)
        telemetry.addData("left vel", shooter.targetVelocityLeft)
        telemetry.addData("right vel", shooter.targetVelocityRight)
        telemetry.update()
        waitForStart()
        runBlocking(Race(
            Forever {
                reads.update()
                intake.update()
                shooter.update()
                telemetry.addData("left shot", shooter.shootCounterLeft)
                telemetry.addData("right shot", shooter.shootCounterRight)
                telemetry.addData("left vel", shooter.targetVelocityLeft)
                telemetry.addData("right vel", shooter.targetVelocityRight)
                telemetry.update()
            },
            Sequence(
                shooter.waitForVelocity(),
                Parallel(
                    intake.releaseDual(),
                    intake.setAdjustThird()
                ),
                Parallel(
                    Instant { intake.behaviour = Intake.IntakeBehaviour.Greedy },
                    Sleep(pusherWait),
                ),
                Parallel(
                    shooter.waitForVelocity(),
                    Instant { intake.behaviour = Intake.IntakeBehaviour.Greedy },
                    Sleep(0.3),
                ),
                intake.releaseDual(),
                Instant { intake.behaviour = Intake.IntakeBehaviour.Grab },
                Sleep(1.0)
            )
        ))
        telemetry.addData("left shot", shooter.shootCounterLeft)
        telemetry.addData("right shot", shooter.shootCounterRight)
        telemetry.update()
        while (opModeIsActive());
    }

}