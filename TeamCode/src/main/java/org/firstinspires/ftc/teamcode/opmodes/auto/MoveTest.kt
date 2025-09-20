package org.firstinspires.ftc.teamcode.opmodes.auto

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive
import org.firstinspires.ftc.teamcode.drivetrain.Pose
import org.firstinspires.ftc.teamcode.util.BulkReads

@TeleOp
class MoveTest2: LinearOpMode() {
    override fun runOpMode() {
        val drive = Drive(hardwareMap)
        val bulkReads = BulkReads(hardwareMap)
        drive.localizer.pose = Pose(0.0, 0.0, 0.0)
        drive.targetPose.x = 24.0
        drive.targetPose.y = 0.0
        waitForStart()
        drive.targetPose.heading = drive.localizer.heading
        while (opModeIsActive()) {
            bulkReads.update()
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