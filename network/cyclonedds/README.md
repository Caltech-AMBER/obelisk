# Cyclone DDS configs for multi-interface / multi-machine Obelisk

Templates for running Obelisk across several networks (e.g. a host bridging two
ethernet segments and wifi). They exist because Cyclone DDS, by default, binds to
a single network interface and uses multicast discovery — which fails for
multi-homed hosts and over wifi.

| File | Runs on | Discovery |
|------|---------|-----------|
| `hub.unicast.xml`   | the multi-interface host | unicast (explicit peers) — **recommended** |
| `hub.hybrid.xml`    | the multi-interface host | multicast on ethernet, unicast on wifi |
| `hub.multicast.xml` | the multi-interface host | full multicast (baseline) |
| `peer.unicast.xml`  | each single-homed peer   | unicast to the hub only |

Each file has placeholder tokens (`HUB_IF_A`, `PEER_A_IP`, …) — fill them from
`ip -br addr` / `ip route` on each machine. Copy a template, edit it, then:

```bash
export OBELISK_CYCLONEDDS_URI=file://$OBELISK_ROOT/network/cyclonedds/<your-file>.xml
obk
```

`obk` exports `CYCLONEDDS_URI` from `OBELISK_CYCLONEDDS_URI` only when it is set;
unset = Obelisk's default DDS behavior, unchanged.

Full walkthrough (with a worked minipc + 2 jetsons + laptop example):
`docs/source/multi_machine.md`.
