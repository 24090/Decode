package org.firstinspires.ftc.teamcode.subsystems.drive.tuners

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.subsystems.controlsystems.Averager
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive.DriveConstants.kS
import org.firstinspires.ftc.teamcode.subsystems.drive.Pose
import org.firstinspires.ftc.teamcode.subsystems.reads.Reads
import kotlin.math.max

@TeleOp(group = "Drive")
@Config
class kVTuner: LinearOpMode() {
    companion object {
        @JvmField var power = 0.1
    }
    override fun runOpMode() {
        val dash = FtcDashboard.getInstance()
        var telemetryPacket = TelemetryPacket()
        var time: Long = 0
        val recordTime = { name:String ->
            val newTime = System.currentTimeMillis()
            telemetry.addData("$name (ms)", newTime - time)
            telemetryPacket.put("$name (ms)", newTime - time)
            time = newTime
        }
        val drive = Drive(hardwareMap)
        val reads = Reads(hardwareMap)
        val velAverager = Averager(100)
        reads.update()
        telemetryPacket.put("velAvg", 0.0)
        telemetryPacket.put("vel", 0.0)
        telemetryPacket.put("maxVel", 0.0)
        telemetryPacket.put("Estimated kV", 0.0)
        dash.sendTelemetryPacket(telemetryPacket)
        var maxVel = 0.0
        waitForStart()
        drive.localizer.pose = Pose(0.0, 0.0, 0.0)
        drive.drive = kS + power
        drive.strafe = 0.0
        drive.turn = 0.0
        drive.setMotorPowers()
        time = System.currentTimeMillis()
        while (opModeIsActive()) {
            reads.update()
            println("test")
            velAverager.update(drive.localizer.xVel)
            maxVel = max(maxVel, velAverager.get())
            recordTime("loop")
            telemetryPacket = TelemetryPacket()
            telemetryPacket.put("velAvg", velAverager.get())
            telemetryPacket.put("vel", drive.localizer.xVel)
            telemetryPacket.put("maxVel", maxVel)
            telemetryPacket.put("Estimated kV", power / maxVel)
            telemetry.update()
            dash.sendTelemetryPacket(telemetryPacket)
        }
        drive.drive = 0.0
        drive.strafe = 0.0
        drive.turn = 0.0
        drive.setMotorPowers()
    }
}