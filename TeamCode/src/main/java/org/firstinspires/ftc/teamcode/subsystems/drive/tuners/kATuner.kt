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
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveVectors.Companion.getTranslationalVectors
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.Pose
import org.firstinspires.ftc.teamcode.subsystems.drive.processTurnDriveStrafe
import org.firstinspires.ftc.teamcode.subsystems.reads.Reads

@TeleOp(group = "Drive")
@Config
class kATuner: LinearOpMode() {
    companion object {
        @JvmField var accelpower = 0.0
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
        var power = kS
        reads.update()
        telemetryPacket.put("avgAccel", 0.0)
        telemetryPacket.put("Estimated kA", 0.0)
        dash.sendTelemetryPacket(telemetryPacket)
        waitForStart()
        drive.localizer.pose = Pose(0.0, 0.0, 0.0)
        drive.follow = {
            getTranslationalVectors(power, 0.0)
        }
        drive.update()
        time = System.currentTimeMillis()
        val startTime = System.currentTimeMillis()
        while (opModeIsActive()) {
            println("test")
            reads.update()
            if (System.currentTimeMillis() > startTime + 100) power = kS + drive.localizer.xVel * kV + accelpower
            drive.update()
            val avgAccel = if (System.currentTimeMillis() > startTime + 100) drive.localizer.xVel/(System.currentTimeMillis() - startTime - 100)*1000 else 0.001
            recordTime("loop")
            telemetryPacket = TelemetryPacket()
            telemetryPacket.put("avgAccel", velAverager.get())
            telemetry.addData("avgAccel", velAverager.get())
            telemetryPacket.put("Estimated kA", accelpower/avgAccel)
            telemetry.addData("Estimated kA", accelpower/avgAccel)
            telemetry.update()
            dash.sendTelemetryPacket(telemetryPacket)
        }
        power = 0.0
        drive.update()
    }
}