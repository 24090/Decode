package org.firstinspires.ftc.teamcode.opmodes.tests

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.opmodes.poses.robotLength
import org.firstinspires.ftc.teamcode.opmodes.poses.robotWidth
import org.firstinspires.ftc.teamcode.subsystems.drive.Pose
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive
import org.firstinspires.ftc.teamcode.subsystems.reads.Reads
import kotlin.math.PI

@TeleOp
class MoveTest2: LinearOpMode() {
    override fun runOpMode() {
        val drive = Drive(hardwareMap)
        val reads = Reads(hardwareMap)
        drive.localizer.pose = Pose(robotLength/2.0, robotWidth/2.0, 0.0)
        drive.targetPose.x = 36.0 + 48.0
        drive.targetPose.y = robotWidth/2.0
        drive.targetPose.heading = PI/2
        reads.update()
        waitForStart()
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