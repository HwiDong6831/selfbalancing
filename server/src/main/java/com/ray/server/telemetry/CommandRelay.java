package com.ray.server.telemetry;

import com.ray.server.dto.FaultCommand;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.stereotype.Component;
import org.springframework.web.socket.TextMessage;
import org.springframework.web.socket.WebSocketSession;
import tools.jackson.core.JacksonException;
import tools.jackson.databind.ObjectMapper;

import java.io.IOException;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

/** 브라우저 → ESP32 역방향 경로의 중개. 양쪽 핸들러가 이 빈만 보게 해 순환 참조를 끊는다. */
@Component
public class CommandRelay {

    private static final Logger log = LoggerFactory.getLogger(CommandRelay.class);

    private final ObjectMapper mapper;

    /** 로봇은 하나다. 최신 세션만 들고 있으면 된다. */
    private volatile WebSocketSession esp;

    /** 채널별 최신 명령. ESP 가 없을 때 더미가 대신 읽는다. */
    private final Map<Integer, FaultCommand> faults = new ConcurrentHashMap<>();

    public CommandRelay(ObjectMapper mapper) {
        this.mapper = mapper;
    }

    public void bind(WebSocketSession session) {
        esp = session;
    }

    public void unbind(WebSocketSession session) {
        if (esp == session) {
            esp = null;
        }
    }

    public void apply(FaultCommand cmd) {
        faults.put(cmd.ch(), cmd);

        WebSocketSession session = esp;
        if (session == null || !session.isOpen()) {
            log.info("ESP 미연결 — 명령을 더미에만 적용: ch{} {}", cmd.ch(), cmd.mode());
            return;
        }
        try {
            synchronized (session) {
                session.sendMessage(new TextMessage(mapper.writeValueAsString(cmd)));
            }
            log.info("결함 주입 전달: ch{} {}", cmd.ch(), cmd.mode());
        } catch (IOException | JacksonException e) {
            log.warn("명령 전달 실패", e);
        }
    }

    public String modeOf(int ch) {
        FaultCommand cmd = faults.get(ch);
        return cmd == null ? FaultCommand.NONE : cmd.mode();
    }

    public double rateOf(int ch) {
        FaultCommand cmd = faults.get(ch);
        return cmd == null ? 0.0 : cmd.rate();
    }
}
