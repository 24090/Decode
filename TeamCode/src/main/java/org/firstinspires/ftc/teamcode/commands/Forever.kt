package org.firstinspires.ftc.teamcode.commands
class Forever(f: () -> Command, name: String = "Forever"): RepeatUntil(f, {false}, name)