package org.firstinspires.ftc.teamcode.subsystems.drive

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.opmodes.poses.robotWidth
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.Pose
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.Vector
import org.firstinspires.ftc.teamcode.subsystems.reads.Reads
import org.firstinspires.ftc.teamcode.util.toDouble

@TeleOp(group = "Drive")
class DirectionDebugger: LinearOpMode() {

    override fun runOpMode() {
        val drive = Drive(hardwareMap)
        drive.localizer.setWheelieBulkreadScope()
        val reads = Reads(hardwareMap)
        val dash = FtcDashboard.getInstance()
        waitForStart()
        drive.localizer.pose = Pose(0.0, 0.0, 0.0) //Pose(robotLength/2.0, robotWidth/2.0, 0.0)
        while (opModeIsActive()){
            reads.update()

            val packet = TelemetryPacket()
            val canvas = packet.fieldOverlay()
            canvas.setStrokeWidth(1)
            canvas.strokeCircle(drive.localizer.pose.x - 72.0, drive.localizer.pose.y, robotWidth/2.0)
            canvas.strokeLine(
                drive.localizer.pose.x - 72.0,
                drive.localizer.pose.y,
                drive.localizer.pose.x - 72.0 + Vector.fromPolar(drive.localizer.heading, robotWidth/2.0).x,
                drive.localizer.pose.y + Vector.fromPolar(drive.localizer.heading, robotWidth/2.0).y
            )

            dash.sendTelemetryPacket(packet)

            drive.setFlPower(gamepad1.x.toDouble())
            drive.setFrPower(gamepad1.y.toDouble())
            drive.setBlPower(gamepad1.a.toDouble())
            drive.setBrPower(gamepad1.b.toDouble())

            telemetry.addData("pinpoint lt", drive.localizer.pinpoint.loopTime)
            telemetry.addLine("Switch directions of backwards motors in Drive.kt")
            telemetry.addLine("X -> FL")
            telemetry.addLine("Y -> FR")
            telemetry.addLine("A -> BL")
            telemetry.addLine("B -> BR")
            telemetry.addLine("Switch directions of backwards encoders in Localizer.kt")
            telemetry.addData("Strafe", drive.localizer.y)
            telemetry.addData("Drive", drive.localizer.x)
            telemetry.addData("Heading", drive.localizer.heading)
            telemetry.addData("Pitch", drive.localizer.pinpoint.getPitch(AngleUnit.RADIANS))
            telemetry.update()
        }
    }
}
