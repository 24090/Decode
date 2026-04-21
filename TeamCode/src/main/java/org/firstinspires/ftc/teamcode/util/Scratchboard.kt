package org.firstinspires.ftc.teamcode.util

/*
 SCRATCHBOARD:
    for running random tests and stuff
 */

import org.firstinspires.ftc.teamcode.opmodes.poses.ShootPose
import org.firstinspires.ftc.teamcode.subsystems.controlsystems.sysid.SysIDRoutine
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.Pose
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.PurePursuitPath
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.Vector
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.followers.HeadingBehaviour
import kotlin.math.PI

class mockSysID() : SysIDRoutine<Unit>(
    name = "mockSysID",
    dataHeaders = arrayOf(),
    initSystem = {},
    updateSignal = {v -> println(v); arrayOf()},
    endCondition = {signal, t -> println(t); false}
)

fun main(){
    mockSysID().runOpMode()
}