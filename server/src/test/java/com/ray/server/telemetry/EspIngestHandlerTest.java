package com.ray.server.telemetry;

import com.ray.server.dto.SensorFrame;
import org.junit.jupiter.api.Test;
import org.springframework.web.socket.TextMessage;
import tools.jackson.databind.ObjectMapper;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;

/** 텔레메트리와 시리얼 로그가 각각 올바른 sink 메서드로 가는지 고정. */
class EspIngestHandlerTest {

    private static final class RecordingSink implements TelemetrySink {
        SensorFrame frame;
        String logLine;

        @Override public void publish(SensorFrame f) { this.frame = f; }
        @Override public void publishLog(String line) { this.logLine = line; }
    }

    private final RecordingSink sink = new RecordingSink();
    private final EspIngestHandler handler = new EspIngestHandler(new ObjectMapper(), sink);

    @Test
    void logMessageGoesToPublishLog() {
        handler.handleTextMessage(null,
                new TextMessage("{\"log\":\"I (1234) MAIN: err: -1.20  rate: 3.40\"}"));

        assertEquals("I (1234) MAIN: err: -1.20  rate: 3.40", sink.logLine);
        assertNull(sink.frame, "로그가 텔레메트리로 잘못 들어가면 안 된다");
    }

    @Test
    void telemetryFrameGoesToPublish() {
        handler.handleTextMessage(null, new TextMessage(SensorFrameParseTest.ESP_JSON));

        assertEquals(123456L, sink.frame.ts());
        assertNull(sink.logLine, "텔레메트리가 로그로 잘못 들어가면 안 된다");
    }
}
