package org.firstinspires.ftc.teamcode.opmodes.tests

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.commands.Forever
import org.firstinspires.ftc.teamcode.commands.Race
import org.firstinspires.ftc.teamcode.commands.Sequence
import org.firstinspires.ftc.teamcode.commands.Sleep
import org.firstinspires.ftc.teamcode.commands.runBlocking
import org.firstinspires.ftc.teamcode.opmodes.commands.releasePattern
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive
import org.firstinspires.ftc.teamcode.subsystems.huskylens.HuskyLens
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake
import org.firstinspires.ftc.teamcode.subsystems.reads.Reads
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter
import org.firstinspires.ftc.teamcode.util.IndexTracker
import org.firstinspires.ftc.teamcode.util.Pattern
import kotlin.math.sqrt

@TeleOp()
class IndexedShootingTest: LinearOpMode(){
    override fun runOpMode() {
        val reads = Reads(hardwareMap)
        val shooter = Shooter(hardwareMap)
        val intake = Intake(hardwareMap)

        var time = System.currentTimeMillis()
        val recordTime = { name:String ->
            val newTime = System.currentTimeMillis()
            telemetry.addData("$name (ms)", newTime - time)
            time = newTime
        }
        val huskyLens = HuskyLens(hardwareMap)
        val indexTracker = IndexTracker()
        indexTracker.rampCount = 0
        indexTracker.pattern = Pattern.PPG
        waitForStart()
        shooter.setTargetVelocityFromDistance(48.0 * sqrt(2.0))
        huskyLens.read()
        runBlocking(
            Race(
                Forever {
                    reads.update()
                    recordTime("reads")
                },
                Sequence(
                    releasePattern(intake, shooter, huskyLens, indexTracker)
                ),
                Forever({
                    shooter.update(); recordTime("shooter")
                    intake.update(); recordTime("intake")
                    telemetry.update()
                }, "Writes")
            )
        )
    }
}