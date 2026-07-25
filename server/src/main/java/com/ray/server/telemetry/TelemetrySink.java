package com.ray.server.telemetry;

import com.ray.server.dto.SensorFrame;

public interface TelemetrySink {

    void publish(SensorFrame frame);

    /** ESP32 시리얼 로그 한 줄. 프레임과 같은 소켓으로 나가고 브라우저가 log 필드로 구분한다. */
    void publishLog(String line);
}
