package org.firstinspires.ftc.teamcode.subsystems.drive.tuners

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.subsystems.controlsystems.StallTest
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive.DriveConstants.kS
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveVectors.Companion.getTranslationalVectors
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.Pose
import org.firstinspires.ftc.teamcode.subsystems.reads.Reads
import kotlin.math.max

@TeleOp(group = "Drive")
@Config
class kVTuner: LinearOpMode() {
    companion object {
        @JvmField var velPower = 0.1
    }
    override fun runOpMode() {
        val telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        var time: Long = 0
        val recordTime = { name:String ->
            val newTime = System.currentTimeMillis()
            telemetry.addData("$name (ms)", newTime - time)
            time = newTime
        }
        val drive = Drive(hardwareMap)
        val reads = Reads(hardwareMap)
        val velStallTest = StallTest(100)
        reads.update()
        telemetry.addData("velAvg", 0.0)
        telemetry.addData("vel", 0.0)
        telemetry.addData("maxVel", 0.0)
        telemetry.addData("Estimated kV", 0.0)
        telemetry.update()
        var maxVel = 0.0
        waitForStart()
        drive.localizer.pose = Pose(0.0, 0.0, 0.0)

        var power = kS + velPower
        drive.follow = {
            getTranslationalVectors(power, 0.0)
        }
        drive.update()
        time = System.currentTimeMillis()
        while (opModeIsActive()) {
            reads.update()
            println("test")
            velStallTest.update(drive.localizer.xVel, System.currentTimeMillis())
            maxVel = max(maxVel, velStallTest.get())
            recordTime("loop")
            telemetry.addData("velAvg", velStallTest.get())
            telemetry.addData("vel", drive.localizer.xVel)
            telemetry.addData("maxVel", maxVel)
            telemetry.addData("Estimated kV", velPower / maxVel)
            telemetry.update()
        }
        power = 0.0
        drive.update()
    }
}