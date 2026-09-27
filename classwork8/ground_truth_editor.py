"""Physical maze Ground Truth editor and 5-cm CSV rasterizer for Classwork 8.

The editable maze is intentionally kept separate from the RoboMaster planner:
only the OFFLINE save-time evaluator ever reads the Ground Truth CSV.
Editor orientation: FRONT = up, RIGHT = right.
Exported occupancy CSV orientation: positive map Y = up, positive map X = right.
"""
from __future__ import annotations

import csv
import json
import math
from pathlib import Path
from typing import Optional, Set, Tuple

from .occupancy_grid import FREE, OCCUPIED as WALL

# Directions within the physical editor: FRONT/top, RIGHT, BACK/bottom, LEFT.
DELTAS = ((-1, 0), (0, 1), (1, 0), (0, -1))


class GroundTruthLayout:
    def __init__(self, rows=5, cols=5, start=None, walls=None):
        self.rows, self.cols = int(rows), int(cols)
        if not (1 <= self.rows <= 30 and 1 <= self.cols <= 30):
            raise ValueError("Physical maze dimensions must be 1..30 logical cells")
        self.start: Optional[Tuple[int, int]] = None
        self.walls: Set[Tuple[int, int, int]] = set()
        if start is not None:
            self.set_start(*start)
        for wall in walls or ():
            self.set_wall(*wall, blocked=True)

    def in_bounds(self, row, col):
        return 0 <= row < self.rows and 0 <= col < self.cols

    def set_start(self, row, col):
        row, col = int(row), int(col)
        if not self.in_bounds(row, col):
            raise ValueError("START must be inside the physical maze")
        self.start = (row, col)

    def _canonical(self, row, col, direction):
        row, col, direction = int(row), int(col), int(direction)
        if not self.in_bounds(row, col) or direction not in (0, 1, 2, 3):
            raise ValueError("Invalid physical wall edge")
        dr, dc = DELTAS[direction]
        other = (row + dr, col + dc)
        if not self.in_bounds(*other):
            return None  # The outer perimeter is always a wall.
        if direction == 0:
            return row - 1, col, 2
        if direction == 3:
            return row, col - 1, 1
        return row, col, direction

    def has_wall(self, row, col, direction):
        edge = self._canonical(row, col, direction)
        return edge is None or edge in self.walls

    def set_wall(self, row, col, direction, blocked=True):
        edge = self._canonical(row, col, direction)
        if edge is None:
            return
        if blocked:
            self.walls.add(edge)
        else:
            self.walls.discard(edge)

    def toggle_wall(self, row, col, direction):
        edge = self._canonical(row, col, direction)
        if edge is None:
            return
        self.set_wall(row, col, direction, edge not in self.walls)

    def wall_edges(self):
        edges = set(self.walls)
        for r in range(self.rows):
            for c in range(self.cols):
                if r == 0:
                    edges.add((r, c, 0))
                if r == self.rows - 1:
                    edges.add((r, c, 2))
                if c == 0:
                    edges.add((r, c, 3))
                if c == self.cols - 1:
                    edges.add((r, c, 1))
        return sorted(edges)

    def cells_per_side(self, cell_size_m, resolution_m):
        if resolution_m <= 0 or cell_size_m <= 0:
            raise ValueError("Cell and occupancy resolution must be positive")
        ratio = float(cell_size_m) / float(resolution_m)
        scale = round(ratio)
        if scale < 2 or not math.isclose(ratio, scale, abs_tol=1e-7):
            raise ValueError("Physical cell size must be a whole number of occupancy pixels (60 cm / 5 cm = 12)")
        return int(scale)

    def rasterize(self, cell_size_m=0.6, resolution_m=0.05, wall_thickness_m=0.05):
        """Create a fully-labelled Ground Truth; screen rows/cols are transposed.

        map.csv columns increase along mission-start FRONT (map +X), whereas
        editor rows go down from FRONT to BACK. map.csv rows go down from map +Y
        (LEFT) to -Y (RIGHT). No fake 60-cm=5-cm conversion is performed.
        """
        scale = self.cells_per_side(cell_size_m, resolution_m)
        if not (0.0 < wall_thickness_m < cell_size_m / 2.0):
            raise ValueError("Wall thickness must be positive and under half a physical cell")
        height, width = self.cols * scale, self.rows * scale
        grid = [[FREE] * width for _ in range(height)]
        half_px = wall_thickness_m / (2.0 * resolution_m)
        eps = 1e-7
        for row, col, direction in self.wall_edges():
            r0, r1 = col * scale, (col + 1) * scale
            c0, c1 = (self.rows - 1 - row) * scale, (self.rows - row) * scale
            if direction in (0, 2):
                boundary = c1 if direction == 0 else c0
                columns = [x for x in range(max(0, int(boundary - half_px - 1)),
                                            min(width, int(boundary + half_px + 2)))
                           if abs((x + .5) - boundary) <= half_px + eps]
                for rr in range(r0, r1):
                    for cc in columns:
                        grid[rr][cc] = WALL
            else:
                boundary = r1 if direction == 1 else r0
                rows = [y for y in range(max(0, int(boundary - half_px - 1)),
                                         min(height, int(boundary + half_px + 2)))
                        if abs((y + .5) - boundary) <= half_px + eps]
                for rr in rows:
                    for cc in range(c0, c1):
                        grid[rr][cc] = WALL
        return grid

    def crop_coordinates(self, config):
        """Locate physical GT in the exported 8-m canvas from the marked START.

        This does not use the robot's SLAM result to choose the offset. Origin
        is the robot START cell centre, heading locked to mission-start yaw.
        """
        if self.start is None:
            raise ValueError("Mark the physical START cell before saving Ground Truth")
        scale = self.cells_per_side(config.cell_size_m, config.resolution_m)
        sr, sc = self.start
        x_min = (sr - self.rows + 0.5) * config.cell_size_m
        y_max = (sc + 0.5) * config.cell_size_m
        # CSV is reverse(grid.matrix()): top is +Y; columns are +X.
        top_raw = (config.map_height_m / 2.0 - y_max) / config.resolution_m
        left_raw = (x_min + config.map_width_m / 2.0) / config.resolution_m
        top, left = int(round(top_raw)), int(round(left_raw))
        if not (math.isclose(top_raw, top, abs_tol=1e-6) and
                math.isclose(left_raw, left, abs_tol=1e-6)):
            raise ValueError("GT boundary does not match the exported 5-cm grid. Check physical dimensions and canvas size.")
        total_rows = int(math.ceil(config.map_height_m / config.resolution_m))
        total_cols = int(math.ceil(config.map_width_m / config.resolution_m))
        gt_rows, gt_cols = self.cols * scale, self.rows * scale
        if top < 0 or left < 0 or top + gt_rows > total_rows or left + gt_cols > total_cols:
            raise ValueError("Physical maze extends outside the working canvas; increase the canvas dimensions.")
        return top, left

    def to_payload(self, config, *, wall_thickness_m=0.05):
        top, left = self.crop_coordinates(config)
        return {
            "format": "classwork8-physical-ground-truth-v1",
            "physical_rows": self.rows,
            "physical_cols": self.cols,
            "start": list(self.start),
            "walls": [list(edge) for edge in sorted(self.walls)],
            "cell_size_m": float(config.cell_size_m),
            "occupancy_resolution_m": float(config.resolution_m),
            "wall_thickness_m": float(wall_thickness_m),
            "crop_top": top,
            "crop_left": left,
            "rotate": 0,
            "orientation": "Editor FRONT up / RIGHT right; CSV rows +Y to -Y, columns -X to +X.",
            "purpose": "Offline ground-truth evaluation only. Never passed to SLAM navigation.",
        }

    @classmethod
    def from_payload(cls, payload):
        if payload.get("format") != "classwork8-physical-ground-truth-v1":
            raise ValueError("Not a Classwork 8 physical Ground Truth layout")
        return cls(payload["physical_rows"], payload["physical_cols"],
                   payload.get("start"), payload.get("walls", ()))

    def save(self, csv_path, config, *, wall_thickness_m=0.05):
        csv_path = Path(csv_path).expanduser()
        if csv_path.suffix.lower() != ".csv":
            raise ValueError("Ground Truth output file must end in .csv")
        payload = self.to_payload(config, wall_thickness_m=wall_thickness_m)
        matrix = self.rasterize(config.cell_size_m, config.resolution_m, wall_thickness_m)
        csv_path.parent.mkdir(parents=True, exist_ok=True)
        with csv_path.open("w", newline="", encoding="utf-8") as stream:
            csv.writer(stream).writerows(matrix)
        csv_path.with_suffix(".json").write_text(
            json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8"
        )
        csv_path.with_suffix(".svg").write_text(self.preview_svg(), encoding="utf-8")
        return csv_path.resolve(), payload

    def preview_svg(self):
        cell, margin = 56, 30
        width, height = 2 * margin + self.cols * cell, 2 * margin + self.rows * cell
        elements = [
            '<svg xmlns="http://www.w3.org/2000/svg" width="{}" height="{}" viewBox="0 0 {} {}">'.format(width,height,width,height),
            '<rect width="100%" height="100%" fill="#f8fafc"/>',
        ]
        for r in range(self.rows):
            for c in range(self.cols):
                x,y=margin+c*cell,margin+r*cell
                elements.append('<rect x="{}" y="{}" width="{}" height="{}" fill="white" stroke="#cbd5e1"/>'.format(x,y,cell,cell))
        for r,c,d in self.wall_edges():
            x,y=margin+c*cell,margin+r*cell
            coords = ((x,y,x+cell,y),(x+cell,y,x+cell,y+cell),
                      (x,y+cell,x+cell,y+cell),(x,y,x,y+cell))[d]
            elements.append('<line x1="{}" y1="{}" x2="{}" y2="{}" stroke="#111827" stroke-width="5"/>'.format(*coords))
        if self.start is not None:
            r,c=self.start
            elements.append('<circle cx="{}" cy="{}" r="15" fill="#16a34a"/>'.format(margin+(c+.5)*cell, margin+(r+.5)*cell))
            elements.append('<text x="{}" y="{}" text-anchor="middle" fill="white" font-family="Arial" font-size="13" font-weight="bold">S</text>'.format(margin+(c+.5)*cell,margin+(r+.5)*cell+5))
        elements.append("</svg>")
        return "\n".join(elements)


class GroundTruthEditor:
    """Modal Tk layout painter opened from V04 pre-mission configuration."""

    def __init__(self, parent, config):
        import tkinter as tk
        from tkinter import filedialog, messagebox, ttk
        self.tk, self.ttk = tk, ttk
        self.filedialog, self.messagebox = filedialog, messagebox
        self.config = config
        self.layout = GroundTruthLayout()
        self.csv_path = Path(config.output_dir) / "ground_truth" / "ground_truth.csv"
        self.saved = False
        self.start_requested = False
        self.wall_thickness_var = tk.StringVar(master=parent, value="5")
        self.rows_var = tk.StringVar(master=parent, value="5")
        self.cols_var = tk.StringVar(master=parent, value="5")
        self.tool_var = tk.StringVar(master=parent, value="wall")
        self.info_var = tk.StringVar(master=parent, value="Draw physical walls and click START (green). Outer walls are fixed.")
        # Re-open a previously authored reference automatically when its
        # editable JSON is available, instead of starting with an empty maze.
        previous_csv = Path(str(getattr(config, "ground_truth_csv", "")))
        previous_json = previous_csv.with_suffix(".json")
        if previous_csv.is_file() and previous_json.is_file():
            try:
                previous = json.loads(previous_json.read_text(encoding="utf-8"))
                if (previous.get("format") == "classwork8-physical-ground-truth-v1"
                    and math.isclose(float(previous.get("cell_size_m")), config.cell_size_m, abs_tol=1e-7)
                    and math.isclose(float(previous.get("occupancy_resolution_m")), config.resolution_m, abs_tol=1e-7)):
                    loaded = GroundTruthLayout.from_payload(previous)
                    self.layout = loaded
                    self.csv_path = previous_csv
                    self.rows_var.set(str(loaded.rows))
                    self.cols_var.set(str(loaded.cols))
                    self.wall_thickness_var.set(str(round(float(previous.get("wall_thickness_m", .05))*100, 2)))
            except (OSError, ValueError, TypeError, KeyError):
                pass
        self.window = tk.Toplevel(parent)
        self.window.title("Classwork 8 - Draw Physical Ground Truth (evaluation only)")
        self.window.geometry("1060x820")
        self.window.minsize(790,640)
        self.window.transient(parent)
        self.window.protocol("WM_DELETE_WINDOW", self.close)
        self._build()
        self.redraw()
        self.window.grab_set()

    def _build(self):
        tk,ttk=self.tk,self.ttk
        outer=ttk.Frame(self.window,padding=12)
        outer.pack(fill="both",expand=True)
        ttk.Label(outer,text="PHYSICAL GROUND TRUTH MAP",font=("Segoe UI",17,"bold")).pack(anchor="w")
        ttk.Label(outer,text="FRONT = ↑    RIGHT = →    Physical cell = {:.0f} cm   |   Export = {:.0f} cm/pixel".format(
            self.config.cell_size_m*100,self.config.resolution_m*100)).pack(anchor="w",pady=(2,8))
        controls=ttk.Frame(outer);controls.pack(fill="x")
        ttk.Label(controls,text="Rows").pack(side="left")
        ttk.Entry(controls,textvariable=self.rows_var,width=5).pack(side="left",padx=5)
        ttk.Label(controls,text="Cols").pack(side="left")
        ttk.Entry(controls,textvariable=self.cols_var,width=5).pack(side="left",padx=5)
        ttk.Button(controls,text="APPLY SIZE",command=self.resize).pack(side="left",padx=7)
        ttk.Label(controls,text="Wall thickness (cm)").pack(side="left",padx=(20,4))
        ttk.Entry(controls,textvariable=self.wall_thickness_var,width=6).pack(side="left")
        tools=ttk.Frame(outer);tools.pack(fill="x",pady=(9,5))
        ttk.Radiobutton(tools,text="Draw / erase internal wall",variable=self.tool_var,value="wall").pack(side="left")
        ttk.Radiobutton(tools,text="Set robot START",variable=self.tool_var,value="start").pack(side="left",padx=14)
        ttk.Button(tools,text="CLEAR INTERNAL WALLS",command=self.clear_walls).pack(side="left",padx=10)
        ttk.Button(tools,text="LOAD EDITABLE MAP (.json)",command=self.load_layout).pack(side="right")
        self.canvas=tk.Canvas(outer,bg="#eef2f7",highlightthickness=1,highlightbackground="#94a3b8")
        self.canvas.pack(fill="both",expand=True,pady=(3,7))
        self.canvas.bind("<Button-1>",self.on_click)
        self.canvas.bind("<Button-3>",self.on_right_click)
        self.canvas.bind("<Configure>",lambda _e:self.redraw())
        ttk.Label(outer,textvariable=self.info_var,foreground="#1d4ed8",wraplength=940).pack(anchor="w",pady=5)
        ttk.Label(outer,text="The robot never receives this map. It is used ONLY after saving SLAM output for Accuracy/Coverage.",foreground="#64748b",wraplength=940).pack(anchor="w")
        buttons=ttk.Frame(outer);buttons.pack(fill="x",pady=(10,0))
        ttk.Button(buttons,text="CANCEL",command=self.close).pack(side="left")
        ttk.Button(buttons,text="SAVE AS...",command=lambda:self.save(choose=True)).pack(side="left",padx=8)
        ttk.Button(buttons,text="SAVE & RETURN",command=self.save_return).pack(side="right")
        ttk.Button(buttons,text="SAVE & START SLAM",command=self.save_start).pack(side="right",padx=8)

    def _geometry(self):
        width=max(300,self.canvas.winfo_width())
        height=max(300,self.canvas.winfo_height())
        size=min((width-64)/self.layout.cols,(height-64)/self.layout.rows)
        return (width-size*self.layout.cols)/2,(height-size*self.layout.rows)/2,size

    def _hit(self,event):
        ox,oy,size=self._geometry()
        c=math.floor((event.x-ox)/size)
        r=math.floor((event.y-oy)/size)
        if not self.layout.in_bounds(r,c):
            return None
        dx=(event.x-ox-c*size)/size
        dy=(event.y-oy-r*size)/size
        edge=min(((dy,0),(1-dx,1),(1-dy,2),(dx,3)))[1]
        return r,c,edge

    def on_click(self,event):
        hit=self._hit(event)
        if hit is None:return
        r,c,edge=hit
        if self.tool_var.get()=="start":
            self.layout.set_start(r,c)
        else:
            self.layout.toggle_wall(r,c,edge)
        self.saved=False
        self.redraw()

    def on_right_click(self,event):
        hit=self._hit(event)
        if hit is not None:
            self.layout.toggle_wall(*hit)
            self.saved=False
            self.redraw()

    def redraw(self):
        if not hasattr(self,"canvas"):return
        cv=self.canvas;cv.delete("all")
        ox,oy,size=self._geometry()
        for r in range(self.layout.rows):
            for c in range(self.layout.cols):
                x,y=ox+c*size,oy+r*size
                cv.create_rectangle(x,y,x+size,y+size,fill="white",outline="#cbd5e1")
                if size>=28:
                    cv.create_text(x+4,y+4,text="{},{}".format(r,c),anchor="nw",fill="#9ca3af",font=("Segoe UI",8))
        for r,c,d in self.layout.wall_edges():
            x,y=ox+c*size,oy+r*size
            coords=((x,y,x+size,y),(x+size,y,x+size,y+size),
                    (x,y+size,x+size,y+size),(x,y,x,y+size))[d]
            cv.create_line(*coords,fill="#111827",width=max(3,min(7,size*.10)),capstyle="round")
        if self.layout.start is not None:
            r,c=self.layout.start
            x,y=ox+(c+.5)*size,oy+(r+.5)*size
            cv.create_oval(x-15,y-15,x+15,y+15,fill="#16a34a",outline="")
            cv.create_text(x,y,text="S",fill="white",font=("Segoe UI",11,"bold"))

    def resize(self):
        try:
            rows,cols=int(self.rows_var.get()),int(self.cols_var.get())
            if (rows,cols)==(self.layout.rows,self.layout.cols):return
            if not self.messagebox.askyesno("Resize physical maze","Changing the size clears walls and START. Continue?",parent=self.window):return
            self.layout=GroundTruthLayout(rows,cols)
            self.saved=False
            self.redraw()
        except (ValueError,TypeError) as exc:
            self.messagebox.showerror("Invalid maze dimensions",str(exc),parent=self.window)

    def clear_walls(self):
        if self.messagebox.askyesno("Clear internal walls","Remove all internal walls and preserve START?",parent=self.window):
            self.layout.walls.clear()
            self.saved=False
            self.redraw()

    def load_layout(self):
        filename=self.filedialog.askopenfilename(parent=self.window,title="Load editable physical Ground Truth",filetypes=[("Ground Truth layout","*.json")])
        if not filename:return
        try:
            payload=json.loads(Path(filename).read_text(encoding="utf-8"))
            if (not math.isclose(float(payload.get("cell_size_m")),self.config.cell_size_m,abs_tol=1e-7) or
                not math.isclose(float(payload.get("occupancy_resolution_m")),self.config.resolution_m,abs_tol=1e-7)):
                raise ValueError("Loaded physical cell size / resolution differs from current V04 settings")
            loaded = GroundTruthLayout.from_payload(payload)
            self.layout = loaded
            self.rows_var.set(str(loaded.rows))
            self.cols_var.set(str(loaded.cols))
            self.wall_thickness_var.set(str(round(float(payload.get("wall_thickness_m",0.05))*100,2)))
            self.csv_path=Path(filename).with_suffix(".csv")
            self.saved=False
            self.info_var.set("Loaded editable Ground Truth: "+str(filename))
            self.redraw()
        except (OSError,ValueError,KeyError,TypeError) as exc:
            self.messagebox.showerror("Load failed",str(exc),parent=self.window)

    def save(self,choose=False):
        try:
            target=self.csv_path
            if choose:
                selected=self.filedialog.asksaveasfilename(parent=self.window,title="Save Ground Truth CSV",
                    initialdir=str(self.csv_path.parent),initialfile=self.csv_path.name,
                    defaultextension=".csv",filetypes=[("Ground Truth CSV","*.csv")])
                if not selected:return False
                target=Path(selected)
            wall_m=float(self.wall_thickness_var.get())/100.0
            path,metadata=self.layout.save(target,self.config,wall_thickness_m=wall_m)
            self.csv_path=path
            self.config.ground_truth_csv=str(path)
            self.config.auto_evaluate_on_save=True
            self.config.evaluation_crop_top=metadata["crop_top"]
            self.config.evaluation_crop_left=metadata["crop_left"]
            self.config.evaluation_rotate_deg=0
            self.saved=True
            self.info_var.set("Saved CSV + editable JSON + preview SVG | auto-alignment top={}, left={} | {}".format(
                metadata["crop_top"],metadata["crop_left"],path))
            return True
        except (ValueError,OSError,TypeError) as exc:
            self.messagebox.showerror("Cannot save Ground Truth",str(exc),parent=self.window)
            return False

    def save_return(self):
        if self.save():self.close()

    def save_start(self):
        if self.save():
            self.start_requested=True
            self.close()

    def close(self):
        self.window.destroy()
