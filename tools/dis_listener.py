#!/usr/bin/env python3
"""dis_listener.py

Standalone IEEE 1278.1 DIS listener for CLEARANCE traffic. Binds the
default multicast group used by ClearanceDISEmitter (224.0.0.1:3000),
decodes Entity State / Fire / Detonation PDUs, and prints one line per
event. No dependencies outside the Python standard library.

Run alongside a CLEARANCE PIE session to prove weapons events + entity
tracking are landing on the wire independently of Wireshark. Great as a
portfolio artefact: the decoder is a short, readable file that shows
you can parse the spec yourself, not just rely on a third-party
dissector.

Usage
-----
    python tools/dis_listener.py                    # 224.0.0.1:3000 (default)
    python tools/dis_listener.py --port 3001        # different port
    python tools/dis_listener.py --group 239.1.2.3  # different group

Options
-------
    --entity-states       print per-tick EntityState PDUs (spammy)
    --only-missiles       print EntityState only for Munition (Kind 2)
    --hex                 include a hex dump of each PDU

Interpreting the output
-----------------------
    [Fire  ] #12  firer=48291 -> target=51204  muni=44177 (AIM/UK/HE/contact)
              at (1.09,-1002.51,0.34) km   vel=(0, 0, 0) m/s
    [State ] entity=44177  Kind 2 Munition (USA AIM-120B) force=Friendly
              at (1.10,-1002.10,0.15) km  vel=(220,-15,340) m/s  mark="MSL_1"
    [Deton ] #12  firer=48291 target=51204 muni=44177  result=1 Entity Impact
              at (1.09,-1002.55,4.20) km

Notes
-----
- Multicast requires IP_ADD_MEMBERSHIP on the receiving socket. Works
  same-box (loopback) because the emitter enables IP_MULTICAST_LOOP.
- Entity numbers are FNV-style hashes of the sim callsigns; the mapping
  is one-way. To resolve back to a name, run the paired script that
  hashes a known callsign list once at startup, or read the sim's
  Output Log for the [ClearanceMissile] Fire event #N line which prints
  both the sim callsign and the DIS event number.
"""

from __future__ import annotations

import argparse
import socket
import struct
import sys
from dataclasses import dataclass


HEADER_SIZE           = 12
PDU_TYPE_ENTITY_STATE = 1
PDU_TYPE_FIRE         = 2
PDU_TYPE_DETONATION   = 3

FORCE_ID_NAMES = {0: "Unknown", 1: "Friendly", 2: "Hostile", 3: "Neutral"}
COUNTRY_NAMES  = {224: "UK", 225: "USA"}
KIND_NAMES     = {1: "Platform", 2: "Munition"}
DOMAIN_NAMES = {
    2: "Air",
    3: "Anti-Air",
}
DETONATION_RESULT_NAMES = {
    0: "Other",
    1: "Entity Impact",
    2: "Entity Proximate Detonation",
    3: "Ground Impact",
    4: "Ground Proximate Detonation",
    5: "Detonation",
    6: "None (miss)",
}


@dataclass
class Header:
    version: int
    exercise: int
    pdu_type: int
    family: int
    timestamp: int
    pdu_length: int


def parse_header(buf: bytes) -> Header:
    v, ex, pt, fam, ts, ln, _pad = struct.unpack(">BBBBIHH", buf[:HEADER_SIZE])
    return Header(v, ex, pt, fam, ts, ln)


def entity_type_str(kind: int, domain: int, country: int,
                    category: int, subcat: int, specific: int, extra: int) -> str:
    country_s = COUNTRY_NAMES.get(country, str(country))
    kind_s    = KIND_NAMES.get(kind, str(kind))
    dom_s     = DOMAIN_NAMES.get(domain, str(domain))
    # Well-known AIM-120B tuple (Kind 2 / Domain 3 / Country 225 / 2/8/3)
    if (kind, domain, country, category, subcat, specific) == (2, 3, 225, 2, 8, 3):
        return f"Kind {kind} Munition ({country_s} AIM-120B)"
    return (f"Kind {kind} {kind_s} / Domain {domain} {dom_s} / "
            f"Country {country_s} / {category}/{subcat}/{specific}/{extra}")


def parse_entity_state(buf: bytes) -> dict:
    """Fields we care about from the 144-byte Entity State PDU."""
    off = HEADER_SIZE
    (site, app, entity, force, _art,
     kind, domain, country, cat, subcat, spec, extra) = struct.unpack_from(
        ">HHHBBBBHBBBB", buf, off)
    # HHHBBBBHBBBB = 6 + 4 + 2 + 4 = 16 bytes (EntityID + ForceId/Art +
    # first EntityType). The earlier off += 18 was an arithmetic slip
    # and shifted every subsequent field by 2 bytes, which is why the
    # decoded Location doubles came out with 10^30-plus exponents. - TripleA
    off += 16
    # skip Alternative Entity Type (8 bytes)
    off += 8
    vx, vy, vz = struct.unpack_from(">fff", buf, off); off += 12
    x, y, z    = struct.unpack_from(">ddd", buf, off); off += 24
    psi, theta, phi = struct.unpack_from(">fff", buf, off); off += 12
    # skip appearance(4) + dead-reckoning(40)
    off += 44
    # marking: 1 charset byte + 11 chars
    charset = buf[off]; off += 1
    marking_raw = buf[off:off+11]; off += 11
    marking = marking_raw.rstrip(b"\x00").decode("ascii", errors="replace")
    return {
        "site": site, "app": app, "entity": entity,
        "force": force,
        "kind": kind, "domain": domain, "country": country,
        "cat": cat, "subcat": subcat, "spec": spec, "extra": extra,
        "vel": (vx, vy, vz), "pos": (x, y, z),
        "ori": (psi, theta, phi),
        "marking": marking,
    }


def parse_fire(buf: bytes) -> dict:
    off = HEADER_SIZE
    # 3 x (site u16 + app u16 + entity u16) = 18 bytes
    (fs, fa, firer, ts_, ta, target, ms_, ma, muni,
     es, ea, event) = struct.unpack_from(">HHHHHHHHHHHH", buf, off)
    off += 24
    (fire_mission,) = struct.unpack_from(">I", buf, off); off += 4
    x, y, z = struct.unpack_from(">ddd", buf, off); off += 24
    (kind, domain, country, muni_kind, _p1, _p2, _p3,
     warhead, fuse, qty, rate) = struct.unpack_from(">BBHBBBBHHHH", buf, off)
    off += 16
    vx, vy, vz, rng = struct.unpack_from(">ffff", buf, off)
    return {
        "firer": firer, "target": target, "muni": muni, "event": event,
        "pos": (x, y, z), "vel": (vx, vy, vz), "range": rng,
        "kind": kind, "domain": domain, "country": country,
        "muni_kind": muni_kind, "warhead": warhead, "fuse": fuse,
    }


def parse_detonation(buf: bytes) -> dict:
    off = HEADER_SIZE
    (fs, fa, firer, ts_, ta, target, ms_, ma, muni,
     es, ea, event) = struct.unpack_from(">HHHHHHHHHHHH", buf, off)
    off += 24
    vx, vy, vz = struct.unpack_from(">fff", buf, off); off += 12
    x, y, z    = struct.unpack_from(">ddd", buf, off); off += 24
    (kind, domain, country, muni_kind, _p1, _p2, _p3,
     warhead, fuse, qty, rate) = struct.unpack_from(">BBHBBBBHHHH", buf, off)
    off += 16
    off += 12   # Location in Entity Coordinates
    result = buf[off]
    return {
        "firer": firer, "target": target, "muni": muni, "event": event,
        "pos": (x, y, z), "vel": (vx, vy, vz),
        "result": result,
    }


def fmt_km(xyz):
    x, y, z = xyz
    return f"({x/1000:+.2f}, {y/1000:+.2f}, {z/1000:+.2f}) km"


def format_fire(e: dict) -> str:
    burst = f"munikind={e['muni_kind']} warhead={e['warhead']} fuse={e['fuse']}"
    return (f"[Fire  ] #{e['event']:<5} firer={e['firer']:>5} -> "
            f"target={e['target']:>5}  muni={e['muni']:>5}  {burst}\n"
            f"           at {fmt_km(e['pos'])}   vel={e['vel']} m/s")


def format_detonation(e: dict) -> str:
    result_name = DETONATION_RESULT_NAMES.get(e["result"], f"code {e['result']}")
    return (f"[Deton ] #{e['event']:<5} firer={e['firer']:>5} target={e['target']:>5} "
            f"muni={e['muni']:>5}  result={e['result']} {result_name}\n"
            f"           at {fmt_km(e['pos'])}   vel={e['vel']} m/s")


def format_state(e: dict) -> str:
    force = FORCE_ID_NAMES.get(e["force"], str(e["force"]))
    et    = entity_type_str(e["kind"], e["domain"], e["country"],
                            e["cat"], e["subcat"], e["spec"], e["extra"])
    mark  = f'  mark="{e["marking"]}"' if e["marking"] else ""
    return (f"[State ] entity={e['entity']:>5}  {et}  force={force}\n"
            f"           at {fmt_km(e['pos'])}  vel={tuple(round(v, 1) for v in e['vel'])} m/s{mark}")


def local_ipv4_interfaces() -> list[str]:
    """Best-effort list of local IPv4 addresses for joining multicast."""
    ips: list[str] = []
    try:
        _, _, addrs = socket.gethostbyname_ex(socket.gethostname())
        ips.extend(a for a in addrs if a and not a.startswith("169.254."))
    except OSError:
        pass
    # loopback last, as a fallback
    if "127.0.0.1" not in ips:
        ips.append("127.0.0.1")
    return ips


def open_socket(group: str, port: int) -> socket.socket:
    """Bind and join the multicast group on every local IPv4 interface.

    Windows' IP_ADD_MEMBERSHIP + INADDR_ANY frequently returns WinError
    10065 ("no route to host") when there's no default multicast route,
    so we enumerate each local interface and try to join on it. As long
    as at least one join succeeds, the socket receives.
    """
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
    except (AttributeError, OSError):
        pass                        # SO_REUSEPORT unavailable on some platforms
    s.bind(("", port))

    joined_on: list[str] = []
    for iface in local_ipv4_interfaces():
        mreq = struct.pack("4s4s", socket.inet_aton(group), socket.inet_aton(iface))
        try:
            s.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
            joined_on.append(iface)
        except OSError as exc:
            print(f"  (skip {iface}: {exc})", file=sys.stderr)

    if not joined_on:
        raise OSError(
            f"Could not join multicast group {group} on any local interface. "
            "If CLEARANCE is emitting to a non-multicast unicast address, "
            "restart the listener with --group <that-address>."
        )
    print(f"Joined multicast group {group} on: {', '.join(joined_on)}", file=sys.stderr)
    return s


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--group", default="224.0.0.1", help="multicast group (default 224.0.0.1)")
    ap.add_argument("--port",  default=3000, type=int, help="UDP port (default 3000)")
    ap.add_argument("--entity-states", action="store_true",
                    help="print per-tick EntityState PDUs (spammy)")
    ap.add_argument("--only-missiles", action="store_true",
                    help="print EntityState only for Munition (Kind 2) traffic")
    ap.add_argument("--hex", action="store_true", help="include hex dump per PDU")
    args = ap.parse_args(argv)

    sock = open_socket(args.group, args.port)
    print(f"Listening on {args.group}:{args.port} (Ctrl+C to stop)")
    print("PDU types recognised: 1 EntityState, 2 Fire, 3 Detonation")
    print("-" * 72)

    try:
        while True:
            buf, _sender = sock.recvfrom(65536)
            if len(buf) < HEADER_SIZE:
                continue
            hdr = parse_header(buf)
            if hdr.version != 7:
                continue        # not DIS 1278.1-2012
            try:
                if hdr.pdu_type == PDU_TYPE_FIRE and len(buf) >= 96:
                    print(format_fire(parse_fire(buf)))
                elif hdr.pdu_type == PDU_TYPE_DETONATION and len(buf) >= 104:
                    print(format_detonation(parse_detonation(buf)))
                elif hdr.pdu_type == PDU_TYPE_ENTITY_STATE and len(buf) >= 144:
                    if not args.entity_states and not args.only_missiles:
                        continue
                    state = parse_entity_state(buf)
                    if args.only_missiles and state["kind"] != 2:
                        continue
                    print(format_state(state))
                if args.hex:
                    print("   " + buf.hex())
            except struct.error as exc:
                print(f"[warn ] parse failed for type={hdr.pdu_type} len={len(buf)}: {exc}",
                      file=sys.stderr)
    except KeyboardInterrupt:
        print("\nStopped.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
