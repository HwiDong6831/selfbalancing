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

/** ESP32 → 서버 수신. DummyTelemetrySource 와 같은 자리에 끼는 생산자다. */
@Component
public class EspIngestHandler extends TextWebSocketHandler {

    private static final Logger log = LoggerFactory.getLogger(EspIngestHandler.class);

    private static final String LOG_PREFIX = "{\"log\":";

    private final ObjectMapper mapper;
    private final TelemetrySink sink;
    private final CommandRelay relay;

    public EspIngestHandler(ObjectMapper mapper, TelemetrySink sink, CommandRelay relay) {
        this.mapper = mapper;
        this.sink = sink;
        this.relay = relay;
    }

    @Override
    public void afterConnectionEstablished(WebSocketSession session) {
        relay.bind(session);   // 역방향(결함 주입) 전송에 쓴다
        log.info("ESP32 connected: {}", session.getId());
    }

    @Override
    public void afterConnectionClosed(WebSocketSession session, CloseStatus status) {
        relay.unbind(session);
        log.info("ESP32 disconnected: {} ({})", session.getId(), status);
    }

    /** 한 소켓으로 오는 텔레메트리와 시리얼 로그를 접두사로 가른다. */
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
            log.warn("ESP message parse failed: {}", e.getOriginalMessage());
        }
    }
}
