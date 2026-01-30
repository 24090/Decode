package org.firstinspires.ftc.teamcode.opmodes.tests

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.Pose
import org.firstinspires.ftc.teamcode.util.storedPose

@TeleOp
class StoredPoseTesting: LinearOpMode() {
    override fun runOpMode() {
        storedPose = storedPose?.plus(Pose(10.0, 10.0, 0.0)) ?: Pose(0.0, 0.0, 0.0)
        waitForStart()
        while (opModeIsActive()){
            telemetry.addData("storedPose", storedPose)
            telemetry.update()
        }
    }

}