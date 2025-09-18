package org.firstinspires.ftc.teamcode.commands
class ForeverCommand(f: () -> Command, name: String = "Forever"): RepeatUntil(f, {false}, name)
fun Forever(f: () -> Unit) = ForeverCommand({Instant(f)}, "Forever")