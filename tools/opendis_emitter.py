#!/usr/bin/env python3
"""opendis_emitter.py

Third-party DIS emitter. Publishes synthetic aircraft as IEEE 1278.1
Entity State PDUs using the open-dis Python library, so a CLEARANCE
receiver (or any other DIS federate) sees a completely-foreign
publisher on its wire. Companion to the two listener scripts:

    dis_listener.py     - hand-rolled decoder in this repo
    opendis_listener.py - third-party library decoder
    opendis_emitter.py  - THIS FILE, third-party emitter

Together the three prove bidirectional IEEE 1278.1 interop:
    CLEARANCE emits -> dis_listener.py + opendis_listener.py decode it
    opendis_emitter.py emits -> CLEARANCE's DISReceiver ingests it

Usage
-----
    pip install opendis
    python tools/opendis_emitter.py                   # 224.0.0.1:3000, 3 aircraft
    python tools/opendis_emitter.py --host 192.168.0.42 --port 3000  # unicast

Aircraft move in a slow circle around a sector origin so they look
alive on a receiving scope, then repeat. Callsigns prefixed EXT so
they're visually obvious as external federates on the CLEARANCE
scope.
"""

from __future__ import annotations

import argparse
import math
import socket
import struct
import sys
import time

try:
    from io import BytesIO
    from opendis.dis7 import EntityStatePdu, EntityID, EntityType, Vector3Double, \
        Vector3Float, EulerAngles, EntityMarking
    from opendis.DataOutputStream import DataOutputStream
except ImportError as exc:
    print(f"ERROR: opendis not installed. Run `pip install opendis` first.\n({exc})",
          file=sys.stderr)
    sys.exit(1)


def hash_callsign(name: str) -> int:
    """Match ClearanceDIS::HashCallsignToEntityNumber (FNV-ish 16-bit fold)."""
    h = 2166136261
    for b in name.encode("ascii"):
        h = ((h ^ b) * 16777619) & 0xFFFFFFFF
    return ((h ^ (h >> 16)) & 0xFFFF) or 1


def make_entity_type(kind: int, domain: int, country: int,
                     category: int, subcategory: int, specific: int) -> EntityType:
    t = EntityType()
    t.entityKind = kind
    t.domain = domain
    t.country = country
    t.category = category
    t.subcategory = subcategory
    t.specific = specific
    t.extra = 0
    return t


def make_pdu(callsign: str, site: int, app: int,
             kind_tuple, force: int,
             x: float, y: float, z: float,
             vx: float, vy: float, vz: float,
             psi: float) -> EntityStatePdu:
    p = EntityStatePdu()
    p.exerciseID = 1
    # opendis leaves protocolVersion=7 correct but doesn't populate the
    # rest of the header. Receivers filter by pduType/protocolFamily so
    # setting these is what actually gets the PDU accepted downstream.
    #   PduType 1        = Entity State
    #   ProtocolFamily 1 = Entity Information
    #   Length 144       = fixed-size EntityState with no articulation params
    p.pduType = 1
    p.protocolFamily = 1
    p.length = 144
    # Base Pdu.serialize writes pduStatus via write_unsigned_byte, so it
    # must be a plain int, not the default PduStatus object. Zero is the
    # "no flags" bit pattern per §7.2.2 and matches what CLEARANCE emits.
    p.pduStatus = 0
    p.padding = 0
    # struct32 fields in the library default to a bytes literal but
    # serialize as unsigned ints; overwrite with plain zero to match. - TripleA
    p.entityAppearance = 0
    p.capabilities = 0

    p.entityID = EntityID()
    p.entityID.siteID = site
    p.entityID.applicationID = app
    p.entityID.entityID = hash_callsign(callsign)

    p.entityType = make_entity_type(*kind_tuple)
    p.alternativeEntityType = make_entity_type(*kind_tuple)

    p.forceId = force

    p.entityLocation = Vector3Double()
    p.entityLocation.x = x
    p.entityLocation.y = y
    p.entityLocation.z = z

    p.entityLinearVelocity = Vector3Float()
    p.entityLinearVelocity.x = vx
    p.entityLinearVelocity.y = vy
    p.entityLinearVelocity.z = vz

    p.entityOrientation = EulerAngles()
    p.entityOrientation.psi = psi
    p.entityOrientation.theta = 0.0
    p.entityOrientation.phi = 0.0

    marking = EntityMarking()
    marking.characterSet = 1
    text = callsign[:11].encode("ascii")
    marking.characters = list(text) + [0] * (11 - len(text))
    p.marking = marking

    return p


def serialize(pdu: EntityStatePdu) -> bytes:
    buf = BytesIO()
    out = DataOutputStream(buf)
    pdu.serialize(out)
    return buf.getvalue()


def pick_egress_interface() -> str:
    """Pick a routable local IPv4 for multicast egress. Windows requires
    IP_MULTICAST_IF to be set explicitly - INADDR_ANY silently causes
    SendTo to fail with WSAEHOSTUNREACH because no default multicast
    route exists. Returns first non-loopback local IP, or loopback."""
    try:
        _, _, addrs = socket.gethostbyname_ex(socket.gethostname())
        for a in addrs:
            if a and not a.startswith("169.254.") and a != "127.0.0.1":
                return a
    except OSError:
        pass
    return "127.0.0.1"


def open_send_socket() -> socket.socket:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    s.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 1)
    s.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 1)
    egress = pick_egress_interface()
    s.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_IF,
                 socket.inet_aton(egress))
    print(f"Emitter egress interface: {egress}", file=sys.stderr)
    return s


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--host", default="224.0.0.1")
    ap.add_argument("--port", default=3000, type=int)
    ap.add_argument("--rate", default=10.0, type=float, help="Hz per aircraft")
    ap.add_argument("--site", default=99, type=int, help="DIS Site ID for external federate")
    ap.add_argument("--app",  default=1,  type=int)
    args = ap.parse_args(argv)

    sock = open_send_socket()
    print(f"Publishing external aircraft to {args.host}:{args.port} "
          f"as Site {args.site}, App {args.app} at {args.rate} Hz per entity", file=sys.stderr)

    # Kind/Domain/Country/Cat/Subcat/Specific tuples for the fake aircraft.
    # All Kind 1 (Platform), Domain 2 (Air) so CLEARANCE routes them as
    # aircraft; distinct Category/Subcategory make them visually
    # distinguishable to a federate that renders by entity type.
    aircraft = [
        # callsign,   type,                             force,  orbit centre (m)   orbit radius (m)
        ("EXT_A320",  (1, 2, 224, 1,  8, 1),  1, ( 30000.0,  30000.0, 10000.0), 15000.0),
        ("EXT_F35",   (1, 2, 225, 1, 22, 5),  1, ( 50000.0, -20000.0,  9500.0), 12000.0),
        ("EXT_MIG29", (1, 2, 222, 1, 22, 8),  2, (-40000.0,  10000.0,  8500.0), 18000.0),
    ]

    period = 1.0 / max(args.rate, 0.1)
    start = 0.0                                # deterministic phase; no Date.now here on purpose
    orbit_period_s = 90.0                      # full lap every 90 s per aircraft
    speed_mps = 220.0                          # ~ Mach 0.65

    try:
        t = 0.0
        while True:
            for callsign, kind_tuple, force, (cx, cy, cz), radius in aircraft:
                # Slow horizontal orbit; keeps them visibly moving on the
                # receiving scope without requiring a scenario.
                angle = 2.0 * math.pi * (t / orbit_period_s)
                x = cx + radius * math.cos(angle)
                y = cy + radius * math.sin(angle)
                z = cz
                vx = -speed_mps * math.sin(angle)
                vy =  speed_mps * math.cos(angle)
                vz = 0.0
                psi = math.atan2(vy, vx)

                pdu = make_pdu(
                    callsign=callsign,
                    site=args.site, app=args.app,
                    kind_tuple=kind_tuple, force=force,
                    x=x, y=y, z=z,
                    vx=vx, vy=vy, vz=vz,
                    psi=psi,
                )
                buf = serialize(pdu)
                sock.sendto(buf, (args.host, args.port))

            print(f"t={t:6.1f}s  sent {len(aircraft)} EntityState PDUs "
                  f"({len(aircraft) * len(buf)} bytes)", file=sys.stderr)
            time.sleep(period)
            t += period
    except KeyboardInterrupt:
        print("\nStopped.", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
