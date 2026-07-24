package com.ray.server.config;

import com.ray.server.telemetry.DashboardWebSocketHandler;
import org.springframework.context.annotation.Configuration;
import org.springframework.web.socket.config.annotation.EnableWebSocket;
import org.springframework.web.socket.config.annotation.WebSocketConfigurer;
import org.springframework.web.socket.config.annotation.WebSocketHandlerRegistry;

/** 대시보드 WebSocket 엔드포인트(/ws/telemetry) 등록. */
@Configuration
@EnableWebSocket
public class WebSocketConfig implements WebSocketConfigurer {

    private final DashboardWebSocketHandler dashboardHandler;

    public WebSocketConfig(DashboardWebSocketHandler dashboardHandler) {
        this.dashboardHandler = dashboardHandler;
    }

    @Override
    public void registerWebSocketHandlers(WebSocketHandlerRegistry registry) {
        registry.addHandler(dashboardHandler, "/ws/telemetry")
                .setAllowedOriginPatterns("*");
    }
}
