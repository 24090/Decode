package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.commands.*
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter
import org.firstinspires.ftc.teamcode.util.BulkReads

@TeleOp
class runmotors : LinearOpMode() {
    override fun runOpMode() {
        val intake = Intake(hardwareMap)
        val shooter = Shooter(hardwareMap)
        val bulkReads = BulkReads(hardwareMap)
        waitForStart()
        shooter.targetVelocity = 1500.0
        while (opModeIsActive()){
            bulkReads.update()
            shooter.update()
        }
    }
}
