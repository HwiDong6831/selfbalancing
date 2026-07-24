package com.ray.server.telemetry;

import com.ray.server.dto.SensorFrame;

public interface TelemetrySink {

    void publish(SensorFrame frame);
}
