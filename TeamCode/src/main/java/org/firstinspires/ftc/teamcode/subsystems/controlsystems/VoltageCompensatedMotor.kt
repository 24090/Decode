package org.firstinspires.ftc.teamcode.subsystems.controlsystems

import com.qualcomm.robotcore.hardware.DcMotorEx
import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx
import org.firstinspires.ftc.teamcode.subsystems.reads.VoltageReader.controlHubVoltage
import org.firstinspires.ftc.teamcode.subsystems.reads.VoltageReader.expansionHubVoltage

fun VoltageCompensatedMotor(motor: DcMotorEx, controlHub: Boolean = false, cachingTolerance: Double = 0.005) =
    VoltageCompensatedMotor(controlHub, CachingDcMotorEx(motor, cachingTolerance))
class VoltageCompensatedMotor(val controlHub: Boolean = true, val motor: CachingDcMotorEx): DcMotorEx by motor {
    val compensationFactor
        get() = (if (controlHub) controlHubVoltage else expansionHubVoltage) * 1.0/14.0

    override fun getPower() = motor.power * compensationFactor

    override fun setPower(power: Double) {
        motor.power = power / compensationFactor
    }
}