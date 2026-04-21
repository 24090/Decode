package org.firstinspires.ftc.teamcode.subsystems.shooter

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.VoltageSensor
import org.firstinspires.ftc.teamcode.opmodes.commands.Robot
import org.firstinspires.ftc.teamcode.subsystems.controlsystems.VoltageCompensatedMotor
import org.firstinspires.ftc.teamcode.subsystems.controlsystems.sysid.MotorSysID
import org.firstinspires.ftc.teamcode.subsystems.controlsystems.sysid.SysIDRoutine
import org.firstinspires.ftc.teamcode.subsystems.reads.VoltageReader.expansionHubVoltage
import kotlin.math.abs

@TeleOp(name = "Shooter SysID")
class ShooterSysId : SysIDRoutine<Robot>(
    "Shooter",
    arrayOf("positionLeft", "positionRight"),
    {
        Robot(hardwareMap, telemetry)
    },
    { signal ->
        reads.updateVoltages()
        reads.bulkReads.update()
        shooter.motorLeft.power = signal
        shooter.motorRight.power = signal
        arrayOf(shooter.motorLeft.currentPosition, shooter.motorRight.currentPosition)
    },
    { signal, t ->
        abs(signal) > expansionHubVoltage/14.0
    },
    dynamicStep = 0.2,
    dynamicStepRate = 1.5,
    quasistaticSlope = 0.1
)


@TeleOp
class shooterLeftSysID: MotorSysID("shooterLeft", {VoltageCompensatedMotor(get(DcMotorEx::class.java, "shooterLeft"), true, 0.02)})