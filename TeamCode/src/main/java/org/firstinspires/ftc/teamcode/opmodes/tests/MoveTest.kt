package org.firstinspires.ftc.teamcode.opmodes.tests

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.commands.*
import org.firstinspires.ftc.teamcode.commands.ForeverCommand
import org.firstinspires.ftc.teamcode.commands.Race
import org.firstinspires.ftc.teamcode.commands.runBlocking
import org.firstinspires.ftc.teamcode.opmodes.poses.robotLength
import org.firstinspires.ftc.teamcode.opmodes.poses.robotWidth
import org.firstinspires.ftc.teamcode.subsystems.drive.Pose
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive
import org.firstinspires.ftc.teamcode.subsystems.reads.Reads
import kotlin.math.PI

@TeleOp
class MoveTest2: LinearOpMode() {
    override fun runOpMode() {
        val dash = FtcDashboard.getInstance()
        var time = System.currentTimeMillis()
        var telemetryPacket = TelemetryPacket()
        val recordTime = { name:String ->

            val newTime = System.currentTimeMillis()
            telemetry.addData("$name (ms)", newTime - time)
            telemetryPacket.put("$name (ms)", newTime - time)
            time = newTime
        }
        val drive = Drive(hardwareMap)
        val reads = Reads(hardwareMap)
        drive.localizer.pose = Pose(0.0, 0.0, 0.0)
        drive.targetPose = Pose(0.0, 0.0, 0.0)
        reads.update()
        waitForStart()
        while (opModeIsActive()) {
            reads.update()
            drive.update()
        }
        while (opModeIsActive()) {
            reads.update()
            drive.update()
            telemetry.addLine("error ${drive.error}")
            telemetry.addLine("dError ${drive.dError}")
            telemetry.addLine("drive ${drive.drive}")
            telemetry.addLine("strafe ${drive.strafe}")
            telemetry.addLine("turn ${drive.turn}")
            telemetry.update()
        }
    }
}