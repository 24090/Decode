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
        drive.localizer.pose = Pose(robotLength/2.0, robotWidth/2.0, 0.0)
        drive.targetPose.x = 36.0
        drive.targetPose.y = robotWidth/2.0
        drive.targetPose.heading = PI/2

        reads.update()

        waitForStart()
        runBlocking(
            Race(
                Forever {
                    recordTime("runblocking")
                    dash.sendTelemetryPacket(telemetryPacket)
                    telemetryPacket = TelemetryPacket()
                    recordTime("packet")
                    reads.update()
                    recordTime("reads")
                },
                Sequence(
                    drive.goToCircle(Pose(0.0, 0.0, 0.0)),
                    Sleep(1.0),
                    drive.goToCircle(Pose(24.0, 0.0, 0.0)),
                    Sleep(1.0),
                    drive.goToCircle(Pose(48.0, 0.0, 0.0)),
                    Sleep(1.0),
                    drive.goToCircle(Pose(24.0, 0.0, 0.0)),
                    Sleep(1.0),
                    drive.goToCircle(Pose(0.0, 0.0, 0.0)),
                    Sleep(1.0),
                    drive.goToCircle(Pose(24.0, 0.0, 0.0)),
                    Sleep(1.0),
                    drive.goToCircle(Pose(48.0, 0.0, 0.0)),
                    Sleep(1.0),
                    drive.goToCircle(Pose(24.0, 0.0, 0.0)),
                    Sleep(1.0),
                    drive.goToCircle(Pose(0.0, 0.0, 0.0)),
                    Sleep(1.0),
                    drive.goToCircle(Pose(24.0, 0.0, 0.0)),
                    Sleep(1.0),
                    drive.goToCircle(Pose(48.0, 0.0, 0.0)),
                    Sleep(1.0),
                    drive.goToCircle(Pose(24.0, 0.0, 0.0)),
                    Sleep(1.0),
                    drive.goToCircle(Pose(0.0, 0.0, 0.0)),
                    Sleep(1.0),
                    drive.goToCircle(Pose(24.0, 0.0, 0.0)),
                    Sleep(1.0),
                    drive.goToCircle(Pose(48.0, 0.0, 0.0)),
                    Sleep(1.0),
                    drive.goToCircle(Pose(24.0, 0.0, 0.0)),
                    Sleep(1.0),
                    drive.goToCircle(Pose(0.0, 0.0, 0.0)),
                    Sleep(1.0),
                    drive.goToCircle(Pose(24.0, 0.0, 0.0)),
                    Sleep(1.0),
                    drive.goToCircle(Pose(48.0, 0.0, 0.0)),
                    Sleep(1.0),
                    drive.goToCircle(Pose(24.0, 0.0, 0.0)),
                    Sleep(1.0),
                    drive.goToCircle(Pose(0.0, 0.0, 0.0)),
                    Sleep(1.0),
                    drive.goToCircle(Pose(24.0, 0.0, 0.0)),
                    Sleep(1.0),
                    drive.goToCircle(Pose(48.0, 0.0, 0.0)),
                    Sleep(1.0),
                    drive.goToCircle(Pose(24.0, 0.0, 0.0)),
                    Sleep(1.0),
                    drive.goToCircle(Pose(0.0, 0.0, 0.0)),
                    Sleep(1.0),
                    drive.goToCircle(Pose(24.0, 0.0, 0.0)),
                    Sleep(1.0),
                    drive.goToCircle(Pose(48.0, 0.0, 0.0)),
                    Sleep(1.0),
                    drive.goToCircle(Pose(24.0, 0.0, 0.0)),
                    Sleep(1.0),
                    drive.goToCircle(Pose(0.0, 0.0, 0.0)),
                    Sleep(1.0),
                    drive.goToCircle(Pose(24.0, 0.0, 0.0)),
                    Sleep(1.0),
                    drive.goToCircle(Pose(48.0, 0.0, 0.0)),
                    Sleep(1.0),
                    drive.goToCircle(Pose(24.0, 0.0, 0.0)),
                    Sleep(1.0)

                ),
                Forever {
                    recordTime("actions")
                    drive.update()
                    recordTime("write")
                    telemetry.update()
                }
            )
        )
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