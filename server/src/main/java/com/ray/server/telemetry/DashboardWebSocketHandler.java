package com.ray.server.telemetry;

import com.ray.server.dto.FaultCommand;
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

import java.io.IOException;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.CopyOnWriteArraySet;

// 접속된 클라이언트 모두에게 센싱 정보 전송
@Component
public class DashboardWebSocketHandler extends TextWebSocketHandler implements TelemetrySink {

    private static final Logger log = LoggerFactory.getLogger(DashboardWebSocketHandler.class);

    private final Set<WebSocketSession> sessions = new CopyOnWriteArraySet<>();
    private final ObjectMapper mapper;
    private final CommandRelay relay;

    public DashboardWebSocketHandler(ObjectMapper mapper, CommandRelay relay) {
        this.mapper = mapper;
        this.relay = relay;
    }

    /** 브라우저가 보내는 건 결함 주입 명령뿐이다. 그 외는 버린다. */
    @Override
    protected void handleTextMessage(WebSocketSession session, TextMessage message) {
        try {
            FaultCommand cmd = mapper.readValue(message.getPayload(), FaultCommand.class);
            if (cmd.isFault()) {
                relay.apply(cmd);
            }
        } catch (JacksonException e) {
            log.warn("dashboard command parse failed: {}", e.getOriginalMessage());
        }
    }

    @Override
    public void afterConnectionEstablished(WebSocketSession session) {
        sessions.add(session);
        log.info("dashboard connected: {} (total {})", session.getId(), sessions.size());
    }

    @Override
    public void afterConnectionClosed(WebSocketSession session, CloseStatus status) {
        sessions.remove(session);
        log.info("dashboard disconnected: {} (total {})", session.getId(), sessions.size());
    }

    @Override
    public void publish(SensorFrame frame) {
        if (sessions.isEmpty()) {
            return;
        }
        try {
            broadcast(mapper.writeValueAsString(frame));
        } catch (JacksonException e) {
            log.warn("frame serialize failed", e);
        }
    }

    @Override
    public void publishLog(String line) {
        if (sessions.isEmpty()) {
            return;
        }
        try {
            broadcast(mapper.writeValueAsString(Map.of("log", line)));
        } catch (JacksonException e) {
            log.warn("log serialize failed", e);
        }
    }

    private void broadcast(String json) {
        TextMessage msg = new TextMessage(json);
        for (WebSocketSession session : sessions) {
            if (!session.isOpen()) {
                sessions.remove(session);
                continue;
            }
            try {
                synchronized (session) {
                    session.sendMessage(msg);
                }
            } catch (IOException e) {
                log.debug("send failed, dropping session {}", session.getId());
                sessions.remove(session);
            }
        }
    }
}
