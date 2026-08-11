#!/usr/bin/env python3
"""opendis_listener.py

Third-party DIS listener that decodes CLEARANCE traffic using the
`open-dis` Python library (a mature, independently-authored
implementation of IEEE 1278.1). Runs alongside `dis_listener.py`
(this repo's from-scratch decoder) so two INDEPENDENT decoders can
agree on the same bytes off the wire. Two-decoder agreement is real
spec-compliance evidence: neither implementation is validating its
own emitter.

Setup
-----
    pip install opendis

Usage
-----
    python tools/opendis_listener.py                    # 224.0.0.1:3000
    python tools/opendis_listener.py --port 3001        # different port
    python tools/opendis_listener.py --entity-states    # include Type 1
    python tools/opendis_listener.py --only-missiles    # Kind 2 only

Output format matches dis_listener.py so a side-by-side terminal
comparison is a direct byte-for-byte check.
"""

from __future__ import annotations

import argparse
import io
import socket
import struct
import sys

try:
    from opendis.PduFactory import createPdu
    from opendis.dis7 import EntityStatePdu, FirePdu, DetonationPdu
except ImportError as exc:                                                # noqa: E722
    print(f"ERROR: opendis not installed. Run `pip install opendis` first.\n({exc})",
          file=sys.stderr)
    sys.exit(1)


FORCE_ID_NAMES = {0: "Unknown", 1: "Friendly", 2: "Hostile", 3: "Neutral"}
COUNTRY_NAMES  = {224: "UK", 225: "USA"}
DETONATION_RESULT_NAMES = {
    0: "Other",
    1: "Entity Impact",
    2: "Entity Proximate Detonation",
    3: "Ground Impact",
    4: "Ground Proximate Detonation",
    5: "Detonation",
    6: "None (miss)",
}


def local_ipv4_interfaces() -> list[str]:
    ips: list[str] = []
    try:
        _, _, addrs = socket.gethostbyname_ex(socket.gethostname())
        ips.extend(a for a in addrs if a and not a.startswith("169.254."))
    except OSError:
        pass
    if "127.0.0.1" not in ips:
        ips.append("127.0.0.1")
    return ips


def open_socket(group: str, port: int) -> socket.socket:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
    except (AttributeError, OSError):
        pass
    s.bind(("", port))
    joined: list[str] = []
    for iface in local_ipv4_interfaces():
        mreq = struct.pack("4s4s", socket.inet_aton(group), socket.inet_aton(iface))
        try:
            s.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
            joined.append(iface)
        except OSError as exc:
            print(f"  (skip {iface}: {exc})", file=sys.stderr)
    if not joined:
        raise OSError(f"Could not join {group} on any local interface.")
    print(f"Joined multicast group {group} on: {', '.join(joined)}", file=sys.stderr)
    return s


def fmt_km(x: float, y: float, z: float) -> str:
    return f"({x/1000:+.2f}, {y/1000:+.2f}, {z/1000:+.2f}) km"


def entity_num(entity_id) -> int:
    """open-dis EntityID uses .siteID / .applicationID / .entityID."""
    return getattr(entity_id, "entityID", 0)


def print_fire(pdu: FirePdu) -> None:
    firer  = entity_num(pdu.firingEntityID)
    target = entity_num(pdu.targetEntityID)
    muni   = entity_num(pdu.munitionExpendableID)  # DIS 7 name for munition entity
    event  = pdu.eventID.eventNumber
    mtype  = pdu.descriptor.munitionType
    loc    = pdu.location
    kind, dom, ctry = mtype.entityKind, mtype.domain, mtype.country
    cat, sub, spec = mtype.category, mtype.subcategory, mtype.specific
    print(f"[Fire  ] #{event:<5} firer={firer:>5} -> target={target:>5}  muni={muni:>5}  "
          f"type=({kind}:{dom}:{ctry}:{cat}:{sub}:{spec})")
    print(f"           at {fmt_km(loc.x, loc.y, loc.z)}")


def print_detonation(pdu: DetonationPdu) -> None:
    firer  = entity_num(pdu.firingEntityID)
    target = entity_num(pdu.targetEntityID)
    muni   = entity_num(pdu.explodingEntityID)     # DIS 7 name for the munition
    event  = pdu.eventID.eventNumber
    result = pdu.detonationResult
    loc    = pdu.location
    result_name = DETONATION_RESULT_NAMES.get(result, f"code {result}")
    print(f"[Deton ] #{event:<5} firer={firer:>5} target={target:>5} muni={muni:>5}  "
          f"result={result} {result_name}")
    print(f"           at {fmt_km(loc.x, loc.y, loc.z)}")


def print_entity_state(pdu: EntityStatePdu, only_missiles: bool) -> None:
    et = pdu.entityType
    if only_missiles and et.entityKind != 2:
        return
    entity = entity_num(pdu.entityID)
    force  = FORCE_ID_NAMES.get(pdu.forceId, str(pdu.forceId))
    ctry   = COUNTRY_NAMES.get(et.country, str(et.country))
    loc    = pdu.entityLocation
    mark   = ""
    try:
        raw = bytes(pdu.marking.characters)
        text = raw.rstrip(b" ").rstrip(b"\x00").decode("ascii", errors="replace")
        if text:
            mark = f'  mark="{text}"'
    except Exception:
        pass
    print(f"[State ] entity={entity:>5}  Kind {et.entityKind}/Dom {et.domain}/"
          f"Country {ctry}/{et.category}/{et.subcategory}/{et.specific}  force={force}")
    print(f"           at {fmt_km(loc.x, loc.y, loc.z)}{mark}")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--group", default="224.0.0.1")
    ap.add_argument("--port",  default=3000, type=int)
    ap.add_argument("--entity-states", action="store_true",
                    help="print per-tick EntityState PDUs (spammy)")
    ap.add_argument("--only-missiles", action="store_true",
                    help="restrict EntityState printing to Munition (Kind 2)")
    args = ap.parse_args(argv)

    sock = open_socket(args.group, args.port)
    print(f"Listening on {args.group}:{args.port} via open-dis (Ctrl+C to stop)")
    print("PDU types recognised: 1 EntityState, 2 Fire, 3 Detonation")
    print("-" * 72)

    try:
        while True:
            buf, _sender = sock.recvfrom(65536)
            if len(buf) < 12:
                continue
            try:
                pdu = createPdu(buf)
            except Exception as exc:                                       # noqa: BLE001
                print(f"[warn ] open-dis parse failed ({exc})", file=sys.stderr)
                continue
            if pdu is None:
                continue
            if isinstance(pdu, FirePdu):
                print_fire(pdu)
            elif isinstance(pdu, DetonationPdu):
                print_detonation(pdu)
            elif isinstance(pdu, EntityStatePdu):
                if args.entity_states or args.only_missiles:
                    print_entity_state(pdu, args.only_missiles)
    except KeyboardInterrupt:
        print("\nStopped.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
