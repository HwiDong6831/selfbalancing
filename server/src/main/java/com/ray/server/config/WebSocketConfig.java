package com.ray.server.config;

import com.ray.server.telemetry.DashboardWebSocketHandler;
import com.ray.server.telemetry.EspIngestHandler;
import org.springframework.context.annotation.Configuration;
import org.springframework.web.socket.config.annotation.EnableWebSocket;
import org.springframework.web.socket.config.annotation.WebSocketConfigurer;
import org.springframework.web.socket.config.annotation.WebSocketHandlerRegistry;

/**
 * WebSocket 엔드포인트 등록.
 *
 *   /ws/telemetry : 서버 → 브라우저 (대시보드 브로드캐스트)
 *   /ws/esp       : ESP32 → 서버   (텔레메트리 수신)
 */
@Configuration
@EnableWebSocket
public class WebSocketConfig implements WebSocketConfigurer {

    private final DashboardWebSocketHandler dashboardHandler;
    private final EspIngestHandler espIngestHandler;

    public WebSocketConfig(DashboardWebSocketHandler dashboardHandler,
                           EspIngestHandler espIngestHandler) {
        this.dashboardHandler = dashboardHandler;
        this.espIngestHandler = espIngestHandler;
    }

    @Override
    public void registerWebSocketHandlers(WebSocketHandlerRegistry registry) {
        registry.addHandler(dashboardHandler, "/ws/telemetry")
                .setAllowedOriginPatterns("*");

        // ESP32 는 브라우저가 아니라 Origin 헤더를 보내지 않는다. 패턴 허용 필요.
        registry.addHandler(espIngestHandler, "/ws/esp")
                .setAllowedOriginPatterns("*");
    }
}
