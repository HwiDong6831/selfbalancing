package com.ray.server.telemetry;

import com.ray.server.dto.EspLog;
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
 * DummyTelemetrySource 와 같은 자리에 끼는 "생산자"라 브로드캐스트/스키마/프론트는
 * 손대지 않는다.
 */
@Component
public class EspIngestHandler extends TextWebSocketHandler {

    private static final Logger log = LoggerFactory.getLogger(EspIngestHandler.class);

    private static final String LOG_PREFIX = "{\"log\":";

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

    /**
     * ESP32 가 한 소켓으로 텔레메트리와 시리얼 로그를 함께 보낸다.
     *
     * 30Hz 로 들어오는 텔레메트리를 두 번 파싱하지 않으려고 트리 대신 접두사로 가른다.
     * 송신 측 포맷을 이쪽이 쥐고 있어 가능 (esp/main/telemetry.c 의 send_pending_logs).
     */
    @Override
    protected void handleTextMessage(WebSocketSession session, TextMessage message) {
        String payload = message.getPayload();
        try {
            if (payload.startsWith(LOG_PREFIX)) {
                sink.publishLog(mapper.readValue(payload, EspLog.class).log());
            } else {
                sink.publish(mapper.readValue(payload, SensorFrame.class));
            }
        } catch (JacksonException e) {
            // 하나가 깨져도 스트림 전체를 끊지 않는다. 30Hz 라 다음 프레임이 곧 온다.
            log.warn("ESP message parse failed: {}", e.getOriginalMessage());
        }
    }
}
