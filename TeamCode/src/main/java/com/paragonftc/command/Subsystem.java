package com.paragonftc.command;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

public interface Subsystem {

    void update(TelemetryPacket packet);
}
