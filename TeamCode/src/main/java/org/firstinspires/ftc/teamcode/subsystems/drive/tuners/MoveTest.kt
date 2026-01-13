package org.firstinspires.ftc.teamcode.subsystems.drive.tuners

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.Pose
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive
import org.firstinspires.ftc.teamcode.subsystems.drive.getScaryPathing
import org.firstinspires.ftc.teamcode.subsystems.drive.scaryPathing
import org.firstinspires.ftc.teamcode.subsystems.reads.Reads

@TeleOp
class MoveTest: LinearOpMode() {
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
        drive.localizer.pose = Pose(0.0, 0.0, 0.0)
        drive.startP2PWithTargetPose(Pose(0.0, 0.0, 0.0))
        reads.update()
        telemetry.addLine("x ${drive.localizer.x}")
        telemetry.addLine("y ${drive.localizer.y}")
        dash.sendTelemetryPacket(telemetryPacket)
        telemetryPacket = TelemetryPacket()
        waitForStart()
        time = System.currentTimeMillis()
        while (opModeIsActive()) {
            reads.update()
            drive.update()
            telemetry.addLine("x ${drive.localizer.x}")
            telemetry.addLine("y ${drive.localizer.y}")
            recordTime("loop")
            telemetry.update()
            dash.sendTelemetryPacket(telemetryPacket)
            telemetryPacket = TelemetryPacket()
        }
    }
}