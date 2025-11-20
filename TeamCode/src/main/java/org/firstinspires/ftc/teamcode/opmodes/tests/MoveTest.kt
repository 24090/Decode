package org.firstinspires.ftc.teamcode.opmodes.tests

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.subsystems.drive.Pose
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive
import org.firstinspires.ftc.teamcode.subsystems.reads.Reads
import kotlin.math.PI

@TeleOp
class MoveTest2: LinearOpMode() {
    override fun runOpMode() {
        val drive = Drive(hardwareMap)
        val reads = Reads(hardwareMap)
        drive.localizer.pose = Pose(0.0, 0.0, 0.0)
        drive.targetPose.x = 0.0
        drive.targetPose.y = 20.0
        drive.targetPose.heading = 0.0
        reads.update()
        waitForStart()
        drive.targetPose.heading = drive.localizer.heading
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