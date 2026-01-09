package org.firstinspires.ftc.teamcode.subsystems.drive.tuners

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.Pose
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive
import org.firstinspires.ftc.teamcode.subsystems.drive.followCurve
import org.firstinspires.ftc.teamcode.subsystems.drive.getFollowCurve
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.CubicHermiteSpline
import org.firstinspires.ftc.teamcode.subsystems.reads.Reads
import kotlin.math.PI

@TeleOp
class PathTest: LinearOpMode() {
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
        drive.localizer.pose = Pose(72.0, 20.0, PI/4)
        drive.follow = getFollowCurve(
            CubicHermiteSpline(
                Pose(72.0, 20.0, PI/4),
                Pose(-10.0, 0.0, 3.0),
                Pose(36.0, 72.0, PI/2),
                Pose(0.0, 80.0, 0.0)
            ),
            drive.localizer
        )
        reads.update()
        telemetry.addLine("x ${drive.localizer.x}")
        telemetry.addLine("y ${drive.localizer.y}")
        dash.sendTelemetryPacket(telemetryPacket)
        telemetryPacket = TelemetryPacket()
        waitForStart()
        time = System.currentTimeMillis()
        while (opModeIsActive()) {
            recordTime("rest")
            reads.update()
            recordTime("reads")

            val driveVectors = drive.follow()
            val leftPowers = driveVectors.getLeftPowers()
            val rightPowers = driveVectors.getRightPowers()

            recordTime("drive calculations")

            drive.flMotor.power = leftPowers.first
            drive.frMotor.power = rightPowers.first
            drive.blMotor.power = leftPowers.second
            drive.brMotor.power = rightPowers.second

            recordTime("drive set")
            telemetry.addLine("x ${drive.localizer.x}")
            telemetry.addLine("y ${drive.localizer.y}")
            telemetry.update()
            dash.sendTelemetryPacket(telemetryPacket)
            telemetryPacket = TelemetryPacket()
        }
    }
}