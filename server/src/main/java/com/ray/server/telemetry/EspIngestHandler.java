package com.ray.server.telemetry;

import com.ray.server.dto.SensorFrame;
import tools.jackson.core.JacksonException;
import tools.jackson.databind.ObjectMapper;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.stereotype.Component;
import org.springframework.web.socket.CloseStatus;
import org.springframework.web.socket.TextMessage;
import org.springframework.web.socket.WebSocketSession;
import org.springframework.web.socket.handler.TextWebSocketHandler;

/**
 * ESP32 → 서버 수신(ingest).
 *
 * 받은 JSON 을 SensorFrame 으로 파싱해 그대로 TelemetrySink 로 넘긴다.
 * DummyTelemetrySource 와 정확히 같은 자리에 끼는 "생산자"이므로,
 * 브로드캐스트/스키마/프론트는 손대지 않는다.
 */
@Component
public class EspIngestHandler extends TextWebSocketHandler {

    private static final Logger log = LoggerFactory.getLogger(EspIngestHandler.class);

    private final ObjectMapper mapper;
    private final TelemetrySink sink;

    public EspIngestHandler(ObjectMapper mapper, TelemetrySink sink) {
        this.mapper = mapper;
        this.sink = sink;
    }

    @Override
    public void afterConnectionEstablished(WebSocketSession session) {
        log.info("ESP32 connected: {}", session.getId());
    }

    @Override
    public void afterConnectionClosed(WebSocketSession session, CloseStatus status) {
        log.info("ESP32 disconnected: {} ({})", session.getId(), status);
    }

    @Override
    protected void handleTextMessage(WebSocketSession session, TextMessage message) {
        try {
            SensorFrame frame = mapper.readValue(message.getPayload(), SensorFrame.class);
            sink.publish(frame);
        } catch (JacksonException e) {
            // 프레임 하나가 깨져도 스트림 전체를 끊지 않는다. 30Hz 라 다음 프레임이 곧 온다.
            log.warn("ESP frame parse failed: {}", e.getOriginalMessage());
        }
    }
}
