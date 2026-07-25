package com.ray.server.dto;

/** ESP32 가 보내는 시리얼 로그 한 줄. 와이어 형식: {"log":"I (1234) MAIN: ..."} */
public record EspLog(String log) {}
