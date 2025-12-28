package org.firstinspires.ftc.teamcode.subsystems.drive.tuners

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.subsystems.controlsystems.Averager
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive.DriveConstants.kS
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive.DriveConstants.kV
import org.firstinspires.ftc.teamcode.subsystems.drive.Pose
import org.firstinspires.ftc.teamcode.subsystems.reads.Reads
import kotlin.math.max

@TeleOp(group = "Drive")
@Config
class kATuner: LinearOpMode() {
    companion object {
        @JvmField var power = 0.0
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
        telemetryPacket.put("avgAccel", 0.0)
        telemetryPacket.put("Estimated kA", 0.0)
        dash.sendTelemetryPacket(telemetryPacket)
        waitForStart()
        drive.localizer.pose = Pose(0.0, 0.0, 0.0)
        drive.drive = kS
        drive.strafe = 0.0
        drive.turn = 0.0
        drive.setMotorPowers()
        time = System.currentTimeMillis()
        val startTime = System.currentTimeMillis()
        while (opModeIsActive()) {
            println("test")
            reads.update()
            if (System.currentTimeMillis() > startTime + 100) drive.drive = kS + drive.localizer.xVel * kV + power
            drive.setMotorPowers()
            val avgAccel = if (System.currentTimeMillis() > startTime + 100) drive.localizer.xVel/(System.currentTimeMillis() - startTime - 100)*1000 else 0.001
            recordTime("loop")
            telemetryPacket = TelemetryPacket()
            telemetryPacket.put("avgAccel", velAverager.get())
            telemetry.addData("avgAccel", velAverager.get())
            telemetryPacket.put("Estimated kA", power/avgAccel)
            telemetry.addData("Estimated kA", power/avgAccel)
            telemetry.update()
            dash.sendTelemetryPacket(telemetryPacket)
        }
        drive.drive = 0.0
        drive.strafe = 0.0
        drive.turn = 0.0
        drive.setMotorPowers()
    }
}