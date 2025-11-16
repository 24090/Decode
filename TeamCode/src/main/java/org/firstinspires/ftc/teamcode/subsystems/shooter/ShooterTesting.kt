package org.firstinspires.ftc.teamcode.subsystems.shooter

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp


@TeleOp
@Config
class ShooterVelocityToPowerTuner(): LinearOpMode(){
    companion object {
        @JvmField var targetVelocity = 1500.0
    }
    override fun runOpMode() {
        val shooter = Shooter(hardwareMap)
        val dash = FtcDashboard.getInstance()
        val p = TelemetryPacket()
        p.put("leftVelocity", 0.0)
        p.put("rightVelocity", 0.0)
        p.put("targetVelocity", 1500.0)
        p.put("powerFeedforward", 0.0)
        dash.sendTelemetryPacket(p)
        waitForStart()
        while (opModeIsActive()){
            if (gamepad1.xWasPressed()) {
                targetVelocity += 50
            }
            if (gamepad1.yWasPressed()) {
                targetVelocity -= 50
            }
            shooter.update()
            shooter.targetVelocity = targetVelocity
            val p = TelemetryPacket()
            p.put("leftVelocity", shooter.motorLeft.velocity)
            p.put("rightVelocity", shooter.motorRight.velocity)
            p.put("powerFeedforward", shooter.velocityToPowerLUT.get(shooter.targetVelocity))
            dash.sendTelemetryPacket(p)
        }
    }
}