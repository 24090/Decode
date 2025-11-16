package org.firstinspires.ftc.teamcode.subsystems.drive

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.drivetrain.Pose
import org.firstinspires.ftc.teamcode.util.Reads

@TeleOp(group = "Drive")
class DirectionDebugger: LinearOpMode() {
    fun Boolean.toDouble() = if (this) 1.0 else 0.0

    override fun runOpMode() {
        val drive = Drive(hardwareMap)
        val reads = Reads(hardwareMap)
        waitForStart()
        drive.localizer.pose = Pose(0.0,0.0,0.0);
        while (opModeIsActive()){
            reads.update()
            drive.localizer.update()
            drive.setFlPower(gamepad1.x.toDouble())
            drive.setFrPower(gamepad1.y.toDouble())
            drive.setBlPower(gamepad1.a.toDouble())
            drive.setBrPower(gamepad1.b.toDouble())

            telemetry.addLine("Switch directions of backwards motors in Drive.kt")
            telemetry.addLine("X -> FL")
            telemetry.addLine("Y -> FR")
            telemetry.addLine("A -> BL")
            telemetry.addLine("B -> BR")
            telemetry.addLine("Switch directions of backwards encoders in Localizer.kt")
            telemetry.addData("Strafe", drive.localizer.y)
            telemetry.addData("Drive", drive.localizer.x)
            telemetry.addData("Heading", drive.localizer.heading)
            telemetry.update()
        }
    }
}
