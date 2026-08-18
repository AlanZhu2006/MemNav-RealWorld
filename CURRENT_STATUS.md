# Current Status

Snapshot: **2026-08-18 UTC**

## Verified

| Gate | Result |
| --- | --- |
| Jetson ↔ RTX 4090 network | Approximately 2.3 ms observed round-trip latency |
| Policy exposure | NavDP <code>8888</code>, MemNav <code>18888</code>, hub <code>18889</code>; loopback-only |
| Unified reset | Returned <code>algo=cec_hybrid_navdp</code> with certificate enabled |
| Real multipart inference | Returned a finite 24-point trajectory |
| First causal frame | No history candidate; exact native NavDP fallback |
| Real D435i dry-run | 38 disabled states over 20 s, zero errors and zero nonzero commands |
| Dry-run inference latency | p50 <code>0.638 s</code>, p95 <code>0.681 s</code>, max <code>0.760 s</code> |
| RGB-D synchronization | p95 skew <code>0.066 s</code>, max <code>0.138 s</code> in the recorded run |
| Tunnel-loss guard | Plan aged out and command stayed zero; service recovered after tunnel restore |
| Go2 bridge defaults | <code>0.30 m/s</code> cap, hardware motion floors and remote priority configured |

The dated evidence and exact observed host commands are preserved in
[REALWORLD_GO2_DUAL_MACHINE_DEPLOYMENT_20260818.md](REALWORLD_GO2_DUAL_MACHINE_DEPLOYMENT_20260818.md).

## Not Yet Verified

- Complete CEC + MemNav + NavDP navigation with the Go2 powered and walking.
- Camera optical-axis to <code>base_link</code> bearing-sign calibration for CEC takeover.
- Long-duration exclusive-GPU latency and p99 characterization.
- Formal first-visit/revisit campaign with frozen starts and success protocol.
- Real-world success rate or SPL. No such result is claimed by this snapshot.

## Next Physical Gate

1. Run GPU and Jetson preflight with the robot disabled.
2. Hold a 10-minute D435i dry-run and inspect RViz/status.
3. Validate bearing sign using a static target on both image sides.
4. Kill the SSH tunnel and confirm zero command within the local age/watchdog bounds.
5. Perform a tethered <code>0.5–1.0 m</code> low-speed run with an onsite operator.
6. Only then begin the frozen first-visit/revisit protocol.

This file describes code and observed validation state; it does not assert
that a physical session is currently armed.
