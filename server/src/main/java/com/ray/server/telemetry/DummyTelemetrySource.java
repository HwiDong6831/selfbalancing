package com.ray.server.telemetry;

import com.ray.server.dto.FaultCommand;
import com.ray.server.dto.SensorFrame;
import org.springframework.boot.autoconfigure.condition.ConditionalOnProperty;
import org.springframework.scheduling.annotation.Scheduled;
import org.springframework.stereotype.Component;

import java.util.ArrayList;
import java.util.List;


/**
 * 테스트용 더미 데이터.
 *
 * <p>평소에는 10초 주기 대본대로 결함을 흘리고, 대시보드에서 결함을 주입하면 대본을 멈추고
 * 지시대로만 낸다. ESP 없이 브라우저에서 전 구간을 검증하기 위한 것이다.
 */
@Component
@ConditionalOnProperty(name = "telemetry.mock.enabled", havingValue = "true", matchIfMissing = true)
public class DummyTelemetrySource {

    private static final int[] CH = {0, 1, 6};

    // 펌웨어 VOTING_TOL_* 를 물리 단위로 옮긴 값
    private static final double TOL_ACC = 1500 / 16384.0;
    private static final double TOL_GYRO = 300 / 131.0;

    private static final double DT = 0.02;   // 50Hz

    private final TelemetrySink sink;
    private final CommandRelay relay;
    private final double[] drift = new double[CH.length];   // 임계값 배수로 누적

    private double v = 0.0;
    private int dir = 1;
    private long ticks = 0;

    public DummyTelemetrySource(TelemetrySink sink, CommandRelay relay) {
        this.sink = sink;
        this.relay = relay;
    }

    @Scheduled(fixedRate = 20)   // 50Hz. 펌웨어 TELEMETRY_US 와 같다
    public void tick() {
        v += dir * 0.2;
        if (v >= 10 || v <= -10) dir = -dir;
        ticks++;

        String[] mode = new String[CH.length];
        boolean manual = false;
        for (int i = 0; i < CH.length; i++) {
            mode[i] = relay.modeOf(CH[i]);
            if (!FaultCommand.NONE.equals(mode[i])) manual = true;
        }
        if (!manual) {
            scripted(mode);
        }

        // 배제 판단: dropout 은 즉시, drift 는 임계값을 넘은 뒤부터.
        // freeze 는 정지 중에 정상값과 구분되지 않으므로 배제되지 않는다 — 실제와 같다.
        var dropped = new ArrayList<Integer>();
        var rejected = new ArrayList<Integer>();
        var sensors = new ArrayList<SensorFrame.Sensor>();

        for (int i = 0; i < CH.length; i++) {
            drift[i] = "drift".equals(mode[i]) ? drift[i] + relay.rateOf(CH[i]) * DT : 0.0;

            if ("dropout".equals(mode[i])) dropped.add(CH[i]);
            if ("dropout".equals(mode[i]) || drift[i] > 1.0) rejected.add(CH[i]);

            double da = drift[i] * TOL_ACC;
            double dg = drift[i] * TOL_GYRO;
            sensors.add(new SensorFrame.Sensor(CH[i],
                    v / 100 + da, 1.0 + v / 500, v / 200,
                    v, v, v + dg,
                    -0.004 + CH[i] * 0.002, 0.006 - CH[i] * 0.001, -1.2 + CH[i] * 0.4,
                    mode[i]));
        }

        var tol = new SensorFrame.Voting.Tol(TOL_ACC, TOL_GYRO);
        var voting = new SensorFrame.Voting(tol,
                vote(rejected, dropped, v / 100, 1.0 + v / 500),
                vote(rejected, dropped, v));
        var balance = new SensorFrame.Balance(v, v, 0.0, v);
        sink.publish(new SensorFrame(System.currentTimeMillis(),
                sensors, voting, balance, new SensorFrame.Encoder(v)));
    }

    /** 수동 주입이 없을 때 도는 10초 주기 대본. */
    private void scripted(String[] mode) {
        long phase = (ticks / 50) % 10;
        if (phase == 6)      mode[2] = "drift";
        else if (phase == 8) mode[1] = "dropout";
    }

    /** 펌웨어 voting_fuse 와 같은 규칙으로 판정을 흉내 낸다. */
    private SensorFrame.Voting.Vote vote(List<Integer> rejected, List<Integer> dropped,
                                         double... val) {
        var used = new ArrayList<Integer>();
        for (int ch : CH) {
            if (!rejected.contains(ch)) used.add(ch);
        }

        String result;
        if (used.size() == CH.length)                        result = "ok";
        else if (used.size() == 2)                           result = "degraded";
        else if (used.size() == 1 && dropped.size() == 2)    result = "single";
        else                                                 result = "fail";

        var v = new ArrayList<Double>();
        if (!"fail".equals(result)) {
            for (double d : val) v.add(d);
        } else {
            used.clear();
        }
        var out = new ArrayList<Integer>();
        for (int ch : CH) {
            if (!used.contains(ch)) out.add(ch);
        }
        return new SensorFrame.Voting.Vote(result, used, out, v);
    }
}
