"""Builds the board with KiCad's Python API: placement, and a two-layer grid maze router (Dijkstra).
Run with KiCad's Python:  "<KiCad>/bin/python.exe" make_board.py
"""
import heapq
import os
import sys
from pathlib import Path

import pcbnew

ROOT = Path(__file__).resolve().parent
LIBS = Path(os.environ.get("KICAD_FOOTPRINTS", Path(sys.executable).parent.parent / "share" / "kicad" / "footprints"))
WIDTH, HEIGHT = 30.0, 20.0                       # board size, mm
STEP, TRACK, VIA, DRILL, CLEAR, EDGE = 0.2, 0.25, 0.6, 0.3, 0.2, 0.6
ORIGIN = 100.0                                   # keeps the board away from the sheet corner

# reference: (library, footprint, value, x, y, rotation, {pad: net})
PARTS = {
    "U1": ("Package_SO", "SOIC-8_3.9x4.9mm_P1.27mm", "LM75B", 15, 10, 0,
           {"1": "SDA", "2": "SCL", "3": "OS", "4": "GND", "5": "GND", "6": "GND", "7": "GND", "8": "VCC"}),
    "C1": ("Capacitor_SMD", "C_0603_1608Metric", "100nF", 20.5, 6.5, 90, {"1": "VCC", "2": "GND"}),
    "R1": ("Resistor_SMD", "R_0603_1608Metric", "4k7", 9, 5, 90, {"1": "VCC", "2": "SDA"}),
    "R2": ("Resistor_SMD", "R_0603_1608Metric", "4k7", 7, 5, 90, {"1": "VCC", "2": "SCL"}),
    "R3": ("Resistor_SMD", "R_0603_1608Metric", "1k", 22, 15, 0, {"1": "VCC", "2": "LED_A"}),
    "D1": ("LED_SMD", "LED_0603_1608Metric", "PWR", 25.5, 15, 180, {"1": "GND", "2": "LED_A"}),
    "J1": ("Connector_PinHeader_2.54mm", "PinHeader_1x04_P2.54mm_Vertical", "I2C", 3.5, 6, 0,
           {"1": "VCC", "2": "GND", "3": "SCL", "4": "SDA"}),
    "J2": ("Connector_PinHeader_2.54mm", "PinHeader_1x01_P2.54mm_Vertical", "ALERT", 11, 16, 0, {"1": "OS"}),
}
LAYERS = (pcbnew.F_Cu, pcbnew.B_Cu)


def point(x, y):
    return pcbnew.VECTOR2I(pcbnew.FromMM(ORIGIN + x), pcbnew.FromMM(ORIGIN + y))


def box(item):
    b = item.GetBoundingBox()
    return (pcbnew.ToMM(b.GetLeft()) - ORIGIN, pcbnew.ToMM(b.GetTop()) - ORIGIN,
            pcbnew.ToMM(b.GetRight()) - ORIGIN, pcbnew.ToMM(b.GetBottom()) - ORIGIN)


def inside(x, y, rect, grow=0.0):
    return rect[0] - grow <= x <= rect[2] + grow and rect[1] - grow <= y <= rect[3] + grow


def build():
    board = pcbnew.BOARD()
    nets, pads = {}, []
    for ref, (lib, name, value, x, y, angle, pins) in PARTS.items():
        footprint = pcbnew.FootprintLoad(str(LIBS / f"{lib}.pretty"), name)
        if footprint is None:
            raise SystemExit(f"footprint not found: {lib}:{name}")
        footprint.SetFPID(pcbnew.LIB_ID(lib, name))
        footprint.SetReference(ref)
        footprint.SetValue(value)
        footprint.SetPosition(point(x, y))
        footprint.SetOrientationDegrees(angle)
        footprint.Reference().SetVisible(False)
        board.Add(footprint)
        for pad in footprint.Pads():
            net = pins[pad.GetNumber()]
            if net not in nets:
                nets[net] = pcbnew.NETINFO_ITEM(board, net)
                board.Add(nets[net])
            pad.SetNet(nets[net])
            layers = [0, 1] if pad.GetAttribute() == pcbnew.PAD_ATTRIB_PTH else [0]
            pads.append((net, box(pad), layers, f"{ref}.{pad.GetNumber()}"))
    for a, b in (((0, 0), (WIDTH, 0)), ((WIDTH, 0), (WIDTH, HEIGHT)), ((WIDTH, HEIGHT), (0, HEIGHT)), ((0, HEIGHT), (0, 0))):
        edge = pcbnew.PCB_SHAPE(board, pcbnew.SHAPE_T_SEGMENT)
        edge.SetStart(point(*a))
        edge.SetEnd(point(*b))
        edge.SetLayer(pcbnew.Edge_Cuts)
        edge.SetWidth(pcbnew.FromMM(0.1))
        board.Add(edge)
    return board, nets, pads


def route_net(net, pads, copper):
    """Connect all pads of one net. copper = list of (net, rect, layers) already routed."""
    columns, rows = int(WIDTH / STEP) + 1, int(HEIGHT / STEP) + 1
    others = [(r, l) for n, r, l, *_ in pads + copper if n != net]
    cache = {}

    def free(c, r, layer):
        key = (c, r, layer)
        if key not in cache:
            x, y = c * STEP, r * STEP
            cache[key] = (EDGE <= x <= WIDTH - EDGE and EDGE <= y <= HEIGHT - EDGE and
                          not any(layer in l and inside(x, y, rect, CLEAR + TRACK / 2 + 0.01) for rect, l in others))
        return cache[key]

    def via_ok(c, r):
        x, y = c * STEP, r * STEP
        return (EDGE + 0.3 <= x <= WIDTH - EDGE - 0.3 and EDGE + 0.3 <= y <= HEIGHT - EDGE - 0.3 and
                not any(inside(x, y, rect, CLEAR + VIA / 2 + 0.01) for rect, _ in others) and
                not any(inside(x, y, rect, VIA / 2) for _, rect, *_ in pads))

    def cells(pad):
        _, rect, layers, _ = pad
        cx, cy = (rect[0] + rect[2]) / 2, (rect[1] + rect[3]) / 2
        found = {(c, r, l) for c in range(columns) for r in range(rows) for l in layers
                 if abs(c * STEP - cx) <= max((rect[2] - rect[0]) / 4, STEP / 2)
                 and abs(r * STEP - cy) <= max((rect[3] - rect[1]) / 4, STEP / 2)}
        if not found:
            raise SystemExit(f"no grid point inside pad {pad[3]}")
        return found

    mine = [p for p in pads if p[0] == net]
    tree, segments, vias = cells(mine[0]), [], []
    for pad in mine[1:]:
        goal = cells(pad)
        queue = [(0, node, None) for node in sorted(tree)]
        heapq.heapify(queue)
        came = {}
        end = None
        while queue:
            cost, node, parent = heapq.heappop(queue)
            if node in came:
                continue
            came[node] = parent
            if node in goal:
                end = node
                break
            c, r, layer = node
            for dc, dr in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nxt = (c + dc, r + dr, layer)
                if 0 <= nxt[0] < columns and 0 <= nxt[1] < rows and nxt not in came and free(*nxt):
                    turn = parent is not None and (c - parent[0], r - parent[1]) != (dc, dr)
                    heapq.heappush(queue, (cost + 1 + 0.2 * turn, nxt, node))
            other = (c, r, 1 - layer)
            if other not in came and via_ok(c, r) and free(*other):
                heapq.heappush(queue, (cost + 12, other, node))
        if end is None:
            raise SystemExit(f"could not route {net} to {pad[3]}")
        while came[end] is not None and end not in tree:
            previous = came[end]
            if previous[2] != end[2]:
                vias.append((end[0] * STEP, end[1] * STEP))
            else:
                segments.append(((previous[0] * STEP, previous[1] * STEP), (end[0] * STEP, end[1] * STEP), end[2]))
            tree.add(end)
            end = previous
        tree |= goal
    return segments, vias


def merge(segments):
    """Join collinear grid steps into long tracks."""
    segments = list(segments)
    changed = True
    while changed:
        changed = False
        ends = {}
        for i, (a, b, layer) in enumerate(segments):
            for p in (a, b):
                ends.setdefault((round(p[0], 3), round(p[1], 3), layer), []).append(i)
        for (x, y, layer), ids in ends.items():
            if len(ids) != 2:
                continue
            far = [p for i in ids for p in segments[i][:2] if (round(p[0], 3), round(p[1], 3)) != (x, y)]
            if len(far) != 2:
                continue
            (ax, ay), (bx, by) = far
            if (abs(ax - x) < 1e-6 and abs(bx - x) < 1e-6) or (abs(ay - y) < 1e-6 and abs(by - y) < 1e-6):
                segments = [s for i, s in enumerate(segments) if i not in ids] + [(far[0], far[1], layer)]
                changed = True
                break
    return segments


def main():
    board, nets, pads = build()
    copper = []
    order = sorted(nets, key=lambda n: (n == "GND", sum(p[0] == n for p in pads), n))
    total_vias = total_tracks = 0
    for net in order:
        if sum(p[0] == net for p in pads) < 2:
            continue
        segments, vias = route_net(net, pads, copper)
        for a, b, layer in merge(segments):
            track = pcbnew.PCB_TRACK(board)
            track.SetStart(point(*a))
            track.SetEnd(point(*b))
            track.SetWidth(pcbnew.FromMM(TRACK))
            track.SetLayer(LAYERS[layer])
            track.SetNet(nets[net])
            board.Add(track)
            total_tracks += 1
        for a, b, layer in segments:
            copper.append((net, (min(a[0], b[0]) - TRACK / 2, min(a[1], b[1]) - TRACK / 2,
                                 max(a[0], b[0]) + TRACK / 2, max(a[1], b[1]) + TRACK / 2), [layer]))
        for x, y in vias:
            via = pcbnew.PCB_VIA(board)
            via.SetPosition(point(x, y))
            via.SetWidth(pcbnew.FromMM(VIA))
            via.SetDrill(pcbnew.FromMM(DRILL))
            via.SetNet(nets[net])
            board.Add(via)
            copper.append((net, (x - VIA / 2, y - VIA / 2, x + VIA / 2, y + VIA / 2), [0, 1]))
            total_vias += 1
    out = ROOT / "board" / "i2c_sensor.kicad_pcb"
    out.parent.mkdir(exist_ok=True)
    pcbnew.SaveBoard(str(out), board)
    print(f"routed {len(order)} nets: {total_tracks} tracks, {total_vias} vias -> {out.name}")


if __name__ == "__main__":
    main()
