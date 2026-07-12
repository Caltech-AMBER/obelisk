# Running Obelisk Across Multiple Machines / Networks

Obelisk uses ROS 2 with the Cyclone DDS middleware (`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`,
set by `obk`). By default Cyclone discovers peers over **multicast on a single
"preferred" network interface** — the one owning the default route. That default
is fine for a single machine or a single flat LAN, but it breaks down in two
common deployment cases:

1. **A host with several interfaces** (e.g. a minipc bridging two ethernet
   segments and wifi) only talks on *one* of them.
2. **Wifi links**, where multicast discovery is unreliable or silently dropped.

This page explains how to configure Cyclone DDS for those cases using the
opt-in hook Obelisk provides. Obelisk's default behavior is unchanged unless you
set the environment variable below.

## The hook: `OBELISK_CYCLONEDDS_URI`

`obk` exports `CYCLONEDDS_URI` from `OBELISK_CYCLONEDDS_URI` when it is set, and
leaves it untouched otherwise. So to point Cyclone at a custom config:

```bash
export OBELISK_CYCLONEDDS_URI=file://$OBELISK_ROOT/network/cyclonedds/hub.unicast.xml
obk          # exports CYCLONEDDS_URI, sources ROS, etc.
```

Because the dev container runs with `network_mode: host` (see
[`docker/docker-compose.yml`](https://github.com/Caltech-AMBER/obelisk)), the
container sees the host's real interfaces (`eno1`, `wlan0`, …), and because
`$OBELISK_ROOT` is bind-mounted at the same path inside and outside the
container, a `file://$OBELISK_ROOT/...` URI resolves identically in both.

To make it permanent, add the `export` to your (bind-mounted) `~/.bashrc`, or run
`obk --permanent` after exporting it so it is baked into the managed block.

## Templates

Ready-to-edit templates live in
[`network/cyclonedds/`](https://github.com/Caltech-AMBER/obelisk/tree/main/network/cyclonedds):

| File | Where it runs | Discovery |
|------|---------------|-----------|
| `hub.unicast.xml`   | the multi-interface host | unicast (explicit peers) — **recommended** |
| `hub.hybrid.xml`    | the multi-interface host | multicast on ethernet, unicast on wifi |
| `hub.multicast.xml` | the multi-interface host | full multicast (baseline) |
| `peer.unicast.xml`  | each single-homed peer   | unicast to the hub only |

Every template has placeholder tokens (`HUB_IF_A`, `PEER_A_IP`, …). Discover the
real values on each machine with:

```bash
ip -br addr     # interface names + IPs
ip route        # which interface owns the default route
```

## Worked example: minipc hub + two jetsons + laptop

Topology (the case this was written for):

```
 subnet A (eth)   subnet B (eth)        subnet C (wifi)
 jetson1 + lidar   jetson2               laptop
        \            |                    /
         \           |                   /
          +------- minipc (HUB: eno1, eno2, wlan0) -------+
```

Each topic has a single publisher and single subscriber, but the graph as a
whole must span all three networks. The minipc is the only multi-homed node.

**Why unicast here.** With fixed hardware and known IPs, multicast's one real
advantage (efficient 1→N fan-out) does not apply, and the wifi link makes
multicast discovery flaky. Unicast discovery is deterministic and behaves the
same on wifi and ethernet, so it is the recommended starting point. Use
`hub.hybrid.xml` only if you later measure that you want ethernet multicast
fan-out.

**On the minipc (hub):** copy `hub.unicast.xml`, then fill in the three
interface names and the three peer IPs:

```bash
export OBELISK_CYCLONEDDS_URI=file://$OBELISK_ROOT/network/cyclonedds/hub.unicast.xml
obk
obk-launch config=<your_config>.yaml
```

**On each peer (jetson1, jetson2, laptop):** copy `peer.unicast.xml`, set its one
interface name and the hub IP *on that peer's subnet* (jetson1 → hub's subnet-A
IP, etc.):

```bash
export OBELISK_CYCLONEDDS_URI=file://$OBELISK_ROOT/network/cyclonedds/peer.unicast.xml
obk
obk-launch config=<peer_config>.yaml
```

**The lidar.** If it is an Ouster/Velodyne/Livox-class sensor it emits raw UDP,
not DDS — a driver node (on jetson1 or the minipc) reads it and republishes as
ROS. In that case the lidar is *not* a DDS participant and needs no Cyclone
config; only the four ROS hosts do.

## Checklist / gotchas

- **Same `ROS_DOMAIN_ID` on every machine.** Note `obk` set up with `--unitree`
  forces `ROS_DOMAIN_ID=2`; make sure all hosts agree.
- **List every interface** the hub must use under `<Interfaces>`. This is the
  fix for "only one network works".
- **Firewall:** allow UDP on 7400–7500 between all hosts (or disable for the
  test).
- **Reachability:** each peer must be able to reach the hub IP it lists. The
  hub lists every peer.
- **Verify multicast** before relying on it: `ros2 multicast receive` on the hub,
  `ros2 multicast send` on a peer. If the hub sees nothing, that link needs
  unicast/`spdp`.

## Validating the choice

To empirically compare `multicast` / `hybrid` / `unicast` on your actual
hardware (discovery time, message rate, latency, drops), use a 50 Hz
timestamped talker on each peer and a listener on the hub, swapping
`OBELISK_CYCLONEDDS_URI` between passes and running `ros2 daemon stop && ros2
daemon start` after each change. Confirm what is on the wire with `tcpdump -i
<iface> -n 'udp and (port 7400 or portrange 7410-7500)'` — multicast destinations
(`239.255.x.x`) should appear only under the multicast/hybrid configs.
