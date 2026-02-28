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
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive.DriveConstants.kV
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveVectors.Companion.getTranslationalVectors
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.Pose
import org.firstinspires.ftc.teamcode.subsystems.reads.Reads

@TeleOp(group = "Drive")
@Config
class kATuner: LinearOpMode() {
    companion object {
        @JvmField var accelpower = 0.0
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
        var power = kS
        reads.update()
        telemetry.addData("avgAccel", 0.0)
        telemetry.addData("Estimated kA", 0.0)
        telemetry.update()
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
            telemetry.addData("avgAccel", velStallTest.get())
            telemetry.addData("Estimated kA", accelpower/avgAccel)
            telemetry.update()
        }
        power = 0.0
        drive.update()
    }
}