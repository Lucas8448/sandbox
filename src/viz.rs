//! Tiny terminal "retro scope" for visualizing the simulation.
//!
//! Two projection modes:
//!
//! * [`Projection::Ortho2D`] — flat XY plot, Y grows upward (physics
//!   convention, not screen convention).
//! * [`Projection::Perspective`] — true 3D camera with depth buffer, ground
//!   reference grid (depth-shaded), and motion trails that stay visible
//!   across frames.
//!
//! Render priority per cell is `Live > Trail > Ground`, so motion trails
//! always read against the grid and the live marker always reads against
//! its own trail.
//!
//! Enabled with the `viz` feature.

use std::io::{Write, stdout};

use crossterm::{
    QueueableCommand, cursor,
    style::{Color, Print, ResetColor, SetForegroundColor},
    terminal::{Clear, ClearType},
};

use crate::math::Vec3;

/// A single body to draw on the scope. `z` is ignored in 2D ortho mode.
#[derive(Debug, Clone, Copy)]
pub struct Marker {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub glyph: char,
    pub color: Color,
}

impl Marker {
    /// 2D constructor (z = 0). Back-compat with the original API.
    pub fn new(x: f64, y: f64, glyph: char) -> Self {
        Self { x, y, z: 0.0, glyph, color: Color::White }
    }

    /// 3D constructor.
    pub fn new3(pos: Vec3, glyph: char) -> Self {
        Self { x: pos.x, y: pos.y, z: pos.z, glyph, color: Color::White }
    }

    pub fn with_color(mut self, color: Color) -> Self {
        self.color = color;
        self
    }

    pub fn pos(&self) -> Vec3 { Vec3::new(self.x, self.y, self.z) }
}

/// A perspective camera. `up` is whatever you call up in your world
/// (typically `Vec3::new(0.0, 1.0, 0.0)`).
#[derive(Debug, Clone, Copy)]
pub struct Camera {
    pub eye: Vec3,
    pub target: Vec3,
    pub up: Vec3,
    /// Vertical field of view, in radians.
    pub fov_y: f64,
    /// Near clipping distance (anything closer is culled).
    pub near: f64,
    /// Far clipping distance (anything further is culled).
    pub far: f64,
    /// Character aspect compensation. Terminal cells are taller than wide
    /// (~2:1), so set this to `2.0` to keep proportions roughly right.
    pub char_aspect: f64,
}

impl Camera {
    pub fn looking_at(eye: Vec3, target: Vec3) -> Self {
        Self {
            eye,
            target,
            up: Vec3::new(0.0, 1.0, 0.0),
            fov_y: 60.0_f64.to_radians(),
            near: 0.1,
            far: 1.0e9,
            char_aspect: 2.0,
        }
    }

    pub fn with_fov_deg(mut self, deg: f64) -> Self {
        self.fov_y = deg.to_radians();
        self
    }

    fn basis(&self) -> (Vec3, Vec3, Vec3) {
        let forward = (self.target - self.eye).normalize();
        let right = forward.cross(self.up).normalize();
        let up = right.cross(forward);
        (right, up, forward)
    }
}

/// Optional reference grid drawn on the world's y=0 plane (3D only).
/// Drawn with `+` at intersections plus `-` along x-lines and `:` along
/// z-lines so the perspective is readable. Color is depth-shaded.
#[derive(Debug, Clone, Copy)]
pub struct GroundGrid {
    pub half_extent: f64,
    pub step: f64,
    /// Near color (closer to camera). Defaults to a soft cyan.
    pub near_color: Color,
    /// Far color (toward the horizon). Defaults to a dim blue-grey.
    pub far_color: Color,
}

impl GroundGrid {
    pub fn new(half_extent: f64, step: f64) -> Self {
        Self {
            half_extent,
            step,
            near_color: Color::Rgb { r: 110, g: 170, b: 210 },
            far_color: Color::Rgb { r: 40, g: 60, b: 90 },
        }
    }
}

/// How world coordinates map onto the character grid.
#[derive(Debug, Clone, Copy)]
pub enum Projection {
    Ortho2D { x_range: (f64, f64), y_range: (f64, f64) },
    Perspective { camera: Camera },
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Kind {
    Empty,
    Ground,
    Trail,
    Live,
}

#[derive(Clone, Copy)]
struct Cell {
    glyph: char,
    color: Color,
    kind: Kind,
    /// Frames since this cell was last "Live". 0 while live; >=1 once it
    /// has aged into a trail.
    age: u8,
}

impl Cell {
    const EMPTY: Cell = Cell {
        glyph: ' ',
        color: Color::Reset,
        kind: Kind::Empty,
        age: 0,
    };
}

/// Maximum age (in frames) a trail cell stays visible before being cleared.
const TRAIL_MAX_AGE: u8 = 12;

/// ASCII oscilloscope-style plotter. 2D or 3D depending on [`Projection`].
pub struct AsciiScope {
    pub width: u16,
    pub height: u16,
    pub projection: Projection,
    pub trails: bool,
    pub ground: Option<GroundGrid>,
    draw_ground_line_2d: bool,
    cells: Vec<Cell>,
    depth: Vec<f64>,
}

impl AsciiScope {
    /// 2D ortho scope (back-compat).
    pub fn new(width: u16, height: u16, x_range: (f64, f64), y_range: (f64, f64)) -> Self {
        let n = (width as usize) * (height as usize);
        Self {
            width,
            height,
            projection: Projection::Ortho2D { x_range, y_range },
            trails: true,
            ground: None,
            draw_ground_line_2d: true,
            cells: vec![Cell::EMPTY; n],
            depth: vec![f64::INFINITY; n],
        }
    }

    /// 3D perspective scope.
    pub fn new_3d(width: u16, height: u16, camera: Camera) -> Self {
        let n = (width as usize) * (height as usize);
        Self {
            width,
            height,
            projection: Projection::Perspective { camera },
            trails: true,
            ground: None,
            draw_ground_line_2d: false,
            cells: vec![Cell::EMPTY; n],
            depth: vec![f64::INFINITY; n],
        }
    }

    pub fn with_trails(mut self, trails: bool) -> Self {
        self.trails = trails;
        self
    }

    pub fn with_ground(mut self, draw_ground: bool) -> Self {
        self.draw_ground_line_2d = draw_ground;
        self
    }

    pub fn with_ground_grid(mut self, grid: GroundGrid) -> Self {
        self.ground = Some(grid);
        self
    }

    pub fn set_camera(&mut self, camera: Camera) {
        if let Projection::Perspective { camera: c } = &mut self.projection {
            *c = camera;
        }
    }

    fn idx(&self, col: u16, row: u16) -> usize {
        (row as usize) * (self.width as usize) + (col as usize)
    }

    fn project(&self, p: Vec3) -> Option<(u16, u16, f64)> {
        match self.projection {
            Projection::Ortho2D { x_range, y_range } => {
                let (xmin, xmax) = x_range;
                let (ymin, ymax) = y_range;
                if p.x < xmin || p.x > xmax || p.y < ymin || p.y > ymax {
                    return None;
                }
                let nx = (p.x - xmin) / (xmax - xmin);
                let ny = (p.y - ymin) / (ymax - ymin);
                let col = (nx * (self.width as f64 - 1.0)).round() as i32;
                let row = ((1.0 - ny) * (self.height as f64 - 1.0)).round() as i32;
                Some((
                    col.clamp(0, self.width as i32 - 1) as u16,
                    row.clamp(0, self.height as i32 - 1) as u16,
                    0.0,
                ))
            }
            Projection::Perspective { camera } => {
                let (right, up, forward) = camera.basis();
                let rel = p - camera.eye;
                let depth = rel.dot(forward);
                if depth < camera.near || depth > camera.far {
                    return None;
                }
                let rx = rel.dot(right);
                let ry = rel.dot(up);
                let focal_y = 1.0 / (camera.fov_y * 0.5).tan();
                let focal_x = focal_y / camera.char_aspect;
                let nx = (rx / depth) * focal_x;
                let ny = (ry / depth) * focal_y;
                if !(-1.0..=1.0).contains(&nx) || !(-1.0..=1.0).contains(&ny) {
                    return None;
                }
                let col = ((nx * 0.5 + 0.5) * (self.width as f64 - 1.0)).round() as i32;
                let row = (((-ny) * 0.5 + 0.5) * (self.height as f64 - 1.0)).round() as i32;
                Some((
                    col.clamp(0, self.width as i32 - 1) as u16,
                    row.clamp(0, self.height as i32 - 1) as u16,
                    depth,
                ))
            }
        }
    }

    /// Wipe everything.
    pub fn clear(&mut self) {
        for c in &mut self.cells {
            *c = Cell::EMPTY;
        }
        for d in &mut self.depth {
            *d = f64::INFINITY;
        }
    }

    /// Begin-frame: age live cells into trails, fade trails, drop the
    /// oldest, and reset transient buffers (depth + ground cells).
    fn tick(&mut self) {
        for c in &mut self.cells {
            match c.kind {
                Kind::Empty => {}
                Kind::Ground => {
                    // Ground is regenerated each frame; clear it now so the
                    // depth buffer starts clean and ground priority is reset.
                    *c = Cell::EMPTY;
                }
                Kind::Live => {
                    // Last frame's live marker becomes a fresh trail.
                    c.kind = Kind::Trail;
                    c.age = 1;
                    let (g, col) = trail_style(c.age);
                    c.glyph = g;
                    c.color = col;
                }
                Kind::Trail => {
                    c.age = c.age.saturating_add(1);
                    if c.age > TRAIL_MAX_AGE {
                        *c = Cell::EMPTY;
                    } else {
                        let (g, col) = trail_style(c.age);
                        c.glyph = g;
                        c.color = col;
                    }
                }
            }
        }
        for d in &mut self.depth {
            *d = f64::INFINITY;
        }
    }

    /// Plot a cell with priority + depth resolution.
    fn put(&mut self, col: u16, row: u16, depth: f64, glyph: char, color: Color, kind: Kind) {
        let i = self.idx(col, row);
        let existing = self.cells[i].kind;
        let priority = |k: Kind| match k {
            Kind::Empty => 0,
            Kind::Ground => 1,
            Kind::Trail => 2,
            Kind::Live => 3,
        };
        let pn = priority(kind);
        let pe = priority(existing);
        if pn > pe {
            self.cells[i] = Cell { glyph, color, kind, age: 0 };
            self.depth[i] = depth;
        } else if pn == pe && depth <= self.depth[i] {
            self.cells[i] = Cell { glyph, color, kind, age: 0 };
            self.depth[i] = depth;
        }
    }

    fn draw_ground(&mut self) {
        match self.projection {
            Projection::Ortho2D { x_range, .. } if self.draw_ground_line_2d => {
                if let Some((_, row, _)) =
                    self.project(Vec3::new(x_range.0, 0.0, 0.0))
                {
                    for col in 0..self.width {
                        self.put(col, row, f64::INFINITY, '=', Color::DarkYellow, Kind::Ground);
                    }
                }
            }
            Projection::Perspective { camera } => {
                if let Some(grid) = self.ground {
                    let h = grid.half_extent;
                    let step = grid.step.max(1e-3);
                    // Used for depth shading.
                    let cam_dist = (camera.eye - camera.target).magnitude().max(1.0);

                    // Walk along Z-lines (constant x) sampling fine points,
                    // alternating glyphs for perspective readability:
                    // `+` at integer-grid intersections, `:` between them on
                    // a Z-line, `-` between them on an X-line.
                    let fine = step / 4.0;

                    // Z-direction lines (each at constant x = i*step):
                    let nx = (h / step).floor() as i32;
                    for ix in -nx..=nx {
                        let x = ix as f64 * step;
                        let mut z = -h;
                        while z <= h + 1e-9 {
                            let on_intersection = (z / step).round() * step;
                            let is_int = (z - on_intersection).abs() < fine * 0.5;
                            let glyph = if is_int { '+' } else { ':' };
                            if let Some((col, row, depth)) =
                                self.project(Vec3::new(x, 0.0, z))
                            {
                                let color = ground_color(depth, cam_dist, &grid);
                                self.put(col, row, depth, glyph, color, Kind::Ground);
                            }
                            z += fine;
                        }
                    }

                    // X-direction lines (each at constant z = j*step), using
                    // `-` between intersections so x and z lines look different.
                    let nz = (h / step).floor() as i32;
                    for iz in -nz..=nz {
                        let z = iz as f64 * step;
                        let mut x = -h;
                        while x <= h + 1e-9 {
                            let on_intersection = (x / step).round() * step;
                            let is_int = (x - on_intersection).abs() < fine * 0.5;
                            let glyph = if is_int { '+' } else { '-' };
                            if let Some((col, row, depth)) =
                                self.project(Vec3::new(x, 0.0, z))
                            {
                                let color = ground_color(depth, cam_dist, &grid);
                                self.put(col, row, depth, glyph, color, Kind::Ground);
                            }
                            x += fine;
                        }
                    }
                }
            }
            _ => {}
        }
    }

    /// Update buffers without flushing to stdout. Useful when compositing
    /// multiple scopes side by side (see [`MultiView`]).
    pub fn update(&mut self, markers: &[Marker]) {
        if self.trails {
            self.tick();
        } else {
            self.clear();
        }

        self.draw_ground();

        for m in markers {
            if let Some((col, row, depth)) = self.project(m.pos()) {
                self.put(col, row, depth, m.glyph, m.color, Kind::Live);
            }
        }
    }

    /// Render one frame to stdout.
    pub fn draw(&mut self, markers: &[Marker]) -> std::io::Result<()> {
        self.update(markers);
        self.flush()
    }

    /// Render the framed scope to a `Vec` of ANSI-colored lines, *without*
    /// any cursor movement. Each line has visible width `self.width + 2`
    /// (left + right border). Length = `self.height + 2`. Used by
    /// [`MultiView`] to stitch panels together.
    pub fn render_lines(&self) -> Vec<String> {
        let mut buf: Vec<u8> = Vec::new();
        self.write_frame(&mut buf).expect("writing to Vec is infallible");
        let s = String::from_utf8(buf).expect("valid utf8");
        s.lines().map(|l| l.to_string()).collect()
    }

    fn write_frame<W: Write>(&self, out: &mut W) -> std::io::Result<()> {
        out.queue(SetForegroundColor(Color::DarkGrey))?;
        out.queue(Print(format!("+{}+\n", "-".repeat(self.width as usize))))?;
        for row in 0..self.height {
            out.queue(SetForegroundColor(Color::DarkGrey))?;
            out.queue(Print("|"))?;
            let mut current = Color::Reset;
            for col in 0..self.width {
                let cell = self.cells[self.idx(col, row)];
                if cell.color != current {
                    out.queue(SetForegroundColor(cell.color))?;
                    current = cell.color;
                }
                out.queue(Print(cell.glyph))?;
            }
            out.queue(SetForegroundColor(Color::DarkGrey))?;
            out.queue(Print("|\n"))?;
        }
        out.queue(SetForegroundColor(Color::DarkGrey))?;
        out.queue(Print(format!("+{}+\n", "-".repeat(self.width as usize))))?;
        out.queue(ResetColor)?;
        Ok(())
    }

    fn flush(&self) -> std::io::Result<()> {
        let mut out = stdout();
        out.queue(cursor::MoveTo(0, 0))?;
        self.write_frame(&mut out)?;
        out.flush()
    }

    pub fn hud(&self, line: &str) -> std::io::Result<()> {
        let mut out = stdout();
        out.queue(Clear(ClearType::CurrentLine))?;
        out.queue(Print(line))?;
        out.queue(Print("\n"))?;
        out.flush()
    }
}

/// Trail glyph + color by age. Newer trails are brighter and use blockier
/// glyphs; older ones fade to dim dots so the live marker pops.
fn trail_style(age: u8) -> (char, Color) {
    match age {
        0 | 1 => ('o', Color::Rgb { r: 200, g: 200, b: 200 }),
        2 | 3 => ('*', Color::Rgb { r: 150, g: 150, b: 150 }),
        4..=6 => ('.', Color::Rgb { r: 110, g: 110, b: 110 }),
        _    => ('.', Color::Rgb { r: 70,  g: 70,  b: 70 }),
    }
}

/// Linearly fade between `near_color` and `far_color` over a sensible
/// distance (proportional to camera distance).
fn ground_color(depth: f64, cam_dist: f64, grid: &GroundGrid) -> Color {
    let max_d = cam_dist * 2.5;
    let t = (depth / max_d).clamp(0.0, 1.0);
    fn rgb(c: Color) -> (u8, u8, u8) {
        match c {
            Color::Rgb { r, g, b } => (r, g, b),
            _ => (128, 128, 128),
        }
    }
    let (nr, ng, nb) = rgb(grid.near_color);
    let (fr, fg, fb) = rgb(grid.far_color);
    let lerp = |a: u8, b: u8| (a as f64 * (1.0 - t) + b as f64 * t).round() as u8;
    Color::Rgb { r: lerp(nr, fr), g: lerp(ng, fg), b: lerp(nb, fb) }
}

/// Convenience: clear screen, home cursor, hide cursor.
pub fn prepare_terminal() -> std::io::Result<()> {
    let mut out = stdout();
    out.queue(Clear(ClearType::All))?;
    out.queue(cursor::MoveTo(0, 0))?;
    out.queue(cursor::Hide)?;
    out.flush()
}

/// Restore cursor visibility.
pub fn restore_terminal() -> std::io::Result<()> {
    let mut out = stdout();
    out.queue(cursor::Show)?;
    out.queue(ResetColor)?;
    out.flush()
}

// ---------------------------------------------------------------------------
// Multi-view (instrument cluster): three 2D ortho panels side-by-side.
// ---------------------------------------------------------------------------

/// Which world axis a panel maps onto its X or Y screen direction.
#[derive(Debug, Clone, Copy)]
pub enum Axis {
    X,
    Y,
    Z,
}

impl Axis {
    fn pick(self, p: Vec3) -> f64 {
        match self {
            Axis::X => p.x,
            Axis::Y => p.y,
            Axis::Z => p.z,
        }
    }
    fn label(self) -> &'static str {
        match self {
            Axis::X => "X",
            Axis::Y => "Y",
            Axis::Z => "Z",
        }
    }
}

struct Panel {
    title: String,
    x_axis: Axis,
    y_axis: Axis,
    x_range: (f64, f64),
    y_range: (f64, f64),
    scope: AsciiScope,
}

/// Three side-by-side 2D ortho panels (SIDE / TOP / FRONT) — like an
/// engineering instrument cluster. Avoids perspective-projection illegibility
/// while still surfacing all three world axes.
pub struct MultiView {
    panels: Vec<Panel>,
    /// Horizontal gap between panels, in characters.
    pub gap: u16,
}

impl MultiView {
    /// Standard three-view layout:
    ///
    /// * SIDE  — X (range) horizontal, Y (altitude) vertical
    /// * TOP   — X (range) horizontal, Z (cross-range) vertical
    /// * FRONT — Z (cross-range) horizontal, Y (altitude) vertical
    pub fn three_view(
        panel_w: u16,
        panel_h: u16,
        x_range: (f64, f64),
        y_range: (f64, f64),
        z_range: (f64, f64),
    ) -> Self {
        let panels = vec![
            Panel {
                title: "SIDE  X->range  Y^alt".into(),
                x_axis: Axis::X,
                y_axis: Axis::Y,
                x_range,
                y_range,
                scope: AsciiScope::new(panel_w, panel_h, x_range, y_range)
                    .with_trails(true)
                    .with_ground(true),
            },
            Panel {
                title: "TOP   X->range  Z^cross".into(),
                x_axis: Axis::X,
                y_axis: Axis::Z,
                x_range,
                y_range: z_range,
                scope: AsciiScope::new(panel_w, panel_h, x_range, z_range)
                    .with_trails(true)
                    .with_ground(false),
            },
            Panel {
                title: "FRONT Z->cross  Y^alt".into(),
                x_axis: Axis::Z,
                y_axis: Axis::Y,
                x_range: z_range,
                y_range,
                scope: AsciiScope::new(panel_w, panel_h, z_range, y_range)
                    .with_trails(true)
                    .with_ground(true),
            },
        ];
        Self { panels, gap: 2 }
    }

    /// Update + render all three panels in one composited frame.
    pub fn draw(&mut self, markers: &[Marker]) -> std::io::Result<()> {
        // Project each marker onto each panel's two world axes and update.
        for panel in &mut self.panels {
            let projected: Vec<Marker> = markers
                .iter()
                .map(|m| {
                    let p = m.pos();
                    Marker {
                        x: panel.x_axis.pick(p),
                        y: panel.y_axis.pick(p),
                        z: 0.0,
                        glyph: m.glyph,
                        color: m.color,
                    }
                })
                .collect();
            panel.scope.update(&projected);
        }

        let renders: Vec<Vec<String>> =
            self.panels.iter().map(|p| p.scope.render_lines()).collect();

        let mut out = stdout();
        out.queue(cursor::MoveTo(0, 0))?;

        // Title row.
        for (i, panel) in self.panels.iter().enumerate() {
            if i > 0 {
                out.queue(Print(" ".repeat(self.gap as usize)))?;
            }
            let total_w = panel.scope.width as usize + 2;
            out.queue(SetForegroundColor(Color::Yellow))?;
            out.queue(Print(center(&panel.title, total_w)))?;
        }
        out.queue(ResetColor)?;
        out.queue(Print("\n"))?;

        // Panel rows.
        let nrows = renders[0].len();
        for r in 0..nrows {
            for (i, lines) in renders.iter().enumerate() {
                if i > 0 {
                    out.queue(Print(" ".repeat(self.gap as usize)))?;
                }
                out.queue(Print(&lines[r]))?;
            }
            out.queue(Print("\n"))?;
        }

        // Axis-range label row (printed under each panel).
        for (i, panel) in self.panels.iter().enumerate() {
            if i > 0 {
                out.queue(Print(" ".repeat(self.gap as usize)))?;
            }
            let total_w = panel.scope.width as usize + 2;
            let lo = fmt_si(panel.x_range.0);
            let hi = fmt_si(panel.x_range.1);
            let mid = format!("{} ({})", panel.x_axis.label(), short_axis_label(panel.x_axis));
            // Build " lo ────── mid ────── hi " padded to total_w.
            let inner = total_w.saturating_sub(2);
            let raw = format!("{} {} {}", lo, mid, hi);
            let line = if raw.len() >= inner {
                raw[..inner].to_string()
            } else {
                let pad = inner - raw.len();
                let left_dashes = pad / 2;
                let right_dashes = pad - left_dashes;
                format!(
                    " {}{}{} {} {}{}{} ",
                    lo,
                    " ",
                    "-".repeat(left_dashes.saturating_sub(2)),
                    mid,
                    "-".repeat(right_dashes.saturating_sub(2)),
                    " ",
                    hi
                )
            };
            out.queue(SetForegroundColor(Color::DarkGrey))?;
            out.queue(Print(truncate_pad(&line, total_w)))?;
        }
        out.queue(ResetColor)?;
        out.queue(Print("\n"))?;

        // y-range label row (just min/max on the left + label on the right).
        for (i, panel) in self.panels.iter().enumerate() {
            if i > 0 {
                out.queue(Print(" ".repeat(self.gap as usize)))?;
            }
            let total_w = panel.scope.width as usize + 2;
            let txt = format!(
                "{} {} {} .. {}",
                panel.y_axis.label(),
                short_axis_label(panel.y_axis),
                fmt_si(panel.y_range.0),
                fmt_si(panel.y_range.1),
            );
            out.queue(SetForegroundColor(Color::DarkGrey))?;
            out.queue(Print(truncate_pad(&txt, total_w)))?;
        }
        out.queue(ResetColor)?;
        out.queue(Print("\n"))?;

        out.flush()
    }

    /// Print a one-line HUD below the cluster.
    pub fn hud(&self, line: &str) -> std::io::Result<()> {
        let mut out = stdout();
        out.queue(Clear(ClearType::CurrentLine))?;
        out.queue(Print(line))?;
        out.queue(Print("\n"))?;
        out.flush()
    }
}

fn center(s: &str, width: usize) -> String {
    if s.len() >= width {
        return s[..width].to_string();
    }
    let pad = width - s.len();
    let left = pad / 2;
    let right = pad - left;
    format!("{}{}{}", " ".repeat(left), s, " ".repeat(right))
}

fn truncate_pad(s: &str, width: usize) -> String {
    if s.len() >= width {
        s[..width].to_string()
    } else {
        format!("{}{}", s, " ".repeat(width - s.len()))
    }
}

fn fmt_si(v: f64) -> String {
    let a = v.abs();
    if a >= 1.0e6 {
        format!("{:.1}M", v / 1.0e6)
    } else if a >= 1.0e3 {
        format!("{:.1}k", v / 1.0e3)
    } else {
        format!("{:.0}", v)
    }
}

fn short_axis_label(a: Axis) -> &'static str {
    match a {
        Axis::X => "range",
        Axis::Y => "alt",
        Axis::Z => "cross",
    }
}

// ---------------------------------------------------------------------------
// Recorder + SVG export
// ---------------------------------------------------------------------------

/// Phase tag attached to each recorded sample. Affects color in SVG output.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TracePhase {
    Boost,
    Coast,
    Impact,
}

#[derive(Debug, Clone, Copy)]
pub struct TraceSample {
    pub t: f64,
    pub pos: Vec3,
    pub phase: TracePhase,
}

/// Records body samples over the lifetime of a simulation. Export as a 4-up
/// SVG (SIDE / TOP / FRONT / summary) after the run.
#[derive(Default, Debug, Clone)]
pub struct Recorder {
    pub samples: Vec<TraceSample>,
    pub label: String,
}

impl Recorder {
    pub fn new(label: impl Into<String>) -> Self {
        Self { samples: Vec::new(), label: label.into() }
    }

    pub fn push(&mut self, t: f64, pos: Vec3, phase: TracePhase) {
        self.samples.push(TraceSample { t, pos, phase });
    }

    /// Write a 2x2 SVG summary to `path`. Auto-fits the data range; pads by
    /// 5%. Pure string output, no external SVG crate.
    pub fn export_svg(&self, path: impl AsRef<std::path::Path>) -> std::io::Result<()> {
        if self.samples.is_empty() {
            return Err(std::io::Error::new(
                std::io::ErrorKind::InvalidInput,
                "no samples recorded",
            ));
        }

        let (xmin, xmax) = bounds(self.samples.iter().map(|s| s.pos.x));
        let (_ymin, ymax) = bounds(self.samples.iter().map(|s| s.pos.y));
        let (zmin, zmax) = bounds(self.samples.iter().map(|s| s.pos.z));

        // Equalize axis scales: every panel's m-per-pixel must match, so we
        // pick the largest data span across X/Y/Z, snap it up to a nice
        // round number, and use a window of that size for every axis.
        // Each axis is anchored so a 0 tick lands on the grid where
        // possible (Y always anchored at 0; X/Z anchored to the nearest
        // tick step around their data midpoint, which usually lands 0 on
        // the grid too).
        let span_x = (xmax - xmin).max(1.0);
        let span_y = (ymax.max(0.0)).max(1.0);
        let span_z = (zmax - zmin).max(1.0);
        let raw_span = span_x.max(span_y).max(span_z) * 1.1;
        let span = nice_ceil(raw_span);
        let step = span / 5.0;
        let snap = |c: f64| (c / step).round() * step;
        let x_center = snap((xmin + xmax) * 0.5);
        let z_center = snap((zmin + zmax) * 0.5);
        let xr = if xmin >= 0.0 {
            (0.0_f64, span)
        } else {
            (x_center - span * 0.5, x_center + span * 0.5)
        };
        let yr = (0.0_f64, span);
        let zr = if zmin >= 0.0 {
            (0.0_f64, span)
        } else {
            (z_center - span * 0.5, z_center + span * 0.5)
        };

        // Layout: 2 columns x 2 rows of square panels so the plot area is
        // square and 1 pixel = the same number of meters on every axis.
        let pw = 560.0_f64;
        let ph = 560.0_f64;
        let margin = 60.0_f64;
        let total_w = (pw * 2.0 + margin * 3.0) as i32;
        let total_h = (ph * 2.0 + margin * 3.0) as i32;

        let mut s = String::new();
        s.push_str(&format!(
            "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"0 0 {w} {h}\" width=\"{w}\" height=\"{h}\">\n",
            w = total_w,
            h = total_h
        ));
        s.push_str("<style>\n");
        s.push_str(
            "  .bg { fill: #0c1116; }\n\
             .panel { fill: #11161c; stroke: #2a3340; stroke-width: 1; }\n\
             .grid { stroke: #1c2430; stroke-width: 1; }\n\
             .axis { stroke: #4a5260; stroke-width: 1; }\n\
             .ground { stroke: #6b5a26; stroke-width: 1.5; stroke-dasharray: 4 3; }\n\
             .tick { fill: #7a8290; font: 11px monospace; }\n\
             .title { fill: #d6deea; font: 14px monospace; font-weight: bold; }\n\
             .subtitle { fill: #7a8290; font: 11px monospace; }\n\
             .boost { stroke: #ff5c5c; stroke-width: 2.2; fill: none; }\n\
             .coast { stroke: #5cff90; stroke-width: 2.2; fill: none; }\n\
             .impact { fill: #ffa040; stroke: #ffa040; }\n\
             .summary { fill: #d6deea; font: 13px monospace; }\n\
             .summary-dim { fill: #7a8290; font: 12px monospace; }\n",
        );
        s.push_str("</style>\n");
        s.push_str(&format!("<rect class=\"bg\" width=\"{}\" height=\"{}\" />\n", total_w, total_h));

        // Helper to draw a panel at (px, py).
        let draw_panel = |s: &mut String, px: f64, py: f64, title: &str,
                          xa: Axis, ya: Axis, xr: (f64, f64), yr: (f64, f64),
                          show_ground: bool| {
            s.push_str(&format!(
                "<g transform=\"translate({px},{py})\">\n",
                px = px,
                py = py
            ));
            s.push_str(&format!(
                "<rect class=\"panel\" width=\"{}\" height=\"{}\" />\n",
                pw, ph
            ));
            s.push_str(&format!(
                "<text class=\"title\" x=\"10\" y=\"18\">{}</text>\n",
                xml_escape(title)
            ));
            s.push_str(&format!(
                "<text class=\"subtitle\" x=\"10\" y=\"32\">{} -&gt; {} (horiz),  {} -&gt; {} (vert)</text>\n",
                xa.label(), short_axis_label(xa), ya.label(), short_axis_label(ya),
            ));

            // Plot area inside the panel. Forced square so the same number
            // of meters spans the same number of pixels on both axes.
            let inset_l = 60.0;
            let inset_r = 20.0;
            let inset_t = 50.0;
            let inset_b = 40.0;
            let plot_size = (pw - inset_l - inset_r).min(ph - inset_t - inset_b);
            let plot_l = inset_l;
            let plot_r = plot_l + plot_size;
            let plot_t = inset_t;
            let plot_b = plot_t + plot_size;
            let map_x = |v: f64| plot_l + (v - xr.0) / (xr.1 - xr.0) * (plot_r - plot_l);
            let map_y = |v: f64| plot_b - (v - yr.0) / (yr.1 - yr.0) * (plot_b - plot_t);

            // Gridlines + ticks (5 each axis).
            for i in 0..=5 {
                let t = i as f64 / 5.0;
                let xv = xr.0 + t * (xr.1 - xr.0);
                let yv = yr.0 + t * (yr.1 - yr.0);
                let xpix = map_x(xv);
                let ypix = map_y(yv);
                s.push_str(&format!(
                    "<line class=\"grid\" x1=\"{:.1}\" y1=\"{:.1}\" x2=\"{:.1}\" y2=\"{:.1}\" />\n",
                    xpix, plot_t, xpix, plot_b
                ));
                s.push_str(&format!(
                    "<line class=\"grid\" x1=\"{:.1}\" y1=\"{:.1}\" x2=\"{:.1}\" y2=\"{:.1}\" />\n",
                    plot_l, ypix, plot_r, ypix
                ));
                s.push_str(&format!(
                    "<text class=\"tick\" x=\"{:.1}\" y=\"{:.1}\" text-anchor=\"middle\">{}</text>\n",
                    xpix, plot_b + 14.0, fmt_si(xv)
                ));
                s.push_str(&format!(
                    "<text class=\"tick\" x=\"{:.1}\" y=\"{:.1}\" text-anchor=\"end\">{}</text>\n",
                    plot_l - 4.0, ypix + 4.0, fmt_si(yv)
                ));
            }
            // Axis lines.
            s.push_str(&format!(
                "<line class=\"axis\" x1=\"{:.1}\" y1=\"{:.1}\" x2=\"{:.1}\" y2=\"{:.1}\" />\n",
                plot_l, plot_t, plot_l, plot_b
            ));
            s.push_str(&format!(
                "<line class=\"axis\" x1=\"{:.1}\" y1=\"{:.1}\" x2=\"{:.1}\" y2=\"{:.1}\" />\n",
                plot_l, plot_b, plot_r, plot_b
            ));
            // Ground line where vertical axis = 0 (only meaningful for Y-vertical panels).
            if show_ground && yr.0 <= 0.0 && yr.1 >= 0.0 {
                let yz = map_y(0.0);
                s.push_str(&format!(
                    "<line class=\"ground\" x1=\"{:.1}\" y1=\"{:.1}\" x2=\"{:.1}\" y2=\"{:.1}\" />\n",
                    plot_l, yz, plot_r, yz
                ));
            }

            // Trajectory polylines, segmented by phase.
            let mut current_phase = self.samples[0].phase;
            let mut current_pts: Vec<(f64, f64)> = Vec::new();
            let flush = |s: &mut String, phase: TracePhase, pts: &[(f64, f64)]| {
                if pts.len() < 2 {
                    return;
                }
                let class = match phase {
                    TracePhase::Boost => "boost",
                    TracePhase::Coast | TracePhase::Impact => "coast",
                };
                let mut d = String::new();
                for (i, (x, y)) in pts.iter().enumerate() {
                    if i == 0 {
                        d.push_str(&format!("M {:.1} {:.1}", x, y));
                    } else {
                        d.push_str(&format!(" L {:.1} {:.1}", x, y));
                    }
                }
                s.push_str(&format!("<path class=\"{}\" d=\"{}\" />\n", class, d));
            };
            for sample in &self.samples {
                let xv = xa.pick(sample.pos);
                let yv = ya.pick(sample.pos);
                let pt = (map_x(xv), map_y(yv));
                if sample.phase != current_phase {
                    // Carry the join point across so segments connect.
                    current_pts.push(pt);
                    flush(s, current_phase, &current_pts);
                    current_phase = sample.phase;
                    current_pts.clear();
                }
                current_pts.push(pt);
            }
            flush(s, current_phase, &current_pts);

            // Impact marker if we recorded one.
            if let Some(last) = self.samples.last() {
                if last.phase == TracePhase::Impact {
                    let xv = xa.pick(last.pos);
                    let yv = ya.pick(last.pos);
                    s.push_str(&format!(
                        "<circle class=\"impact\" cx=\"{:.1}\" cy=\"{:.1}\" r=\"5\" />\n",
                        map_x(xv), map_y(yv)
                    ));
                }
            }

            s.push_str("</g>\n");
        };

        let p_side = (margin, margin);
        let p_top = (margin * 2.0 + pw, margin);
        let p_front = (margin, margin * 2.0 + ph);
        let p_summary = (margin * 2.0 + pw, margin * 2.0 + ph);

        draw_panel(&mut s, p_side.0, p_side.1, "SIDE  X x Y", Axis::X, Axis::Y, xr, yr, true);
        draw_panel(&mut s, p_top.0, p_top.1, "TOP   X x Z", Axis::X, Axis::Z, xr, zr, false);
        draw_panel(&mut s, p_front.0, p_front.1, "FRONT Z x Y", Axis::Z, Axis::Y, zr, yr, true);

        // Summary panel.
        let last = self.samples.last().unwrap();
        let max_alt = self
            .samples
            .iter()
            .map(|s| s.pos.y)
            .fold(f64::NEG_INFINITY, f64::max);
        let ground_range = (last.pos.x.powi(2) + last.pos.z.powi(2)).sqrt();
        let label = if self.label.is_empty() {
            "Trajectory".to_string()
        } else {
            self.label.clone()
        };
        s.push_str(&format!(
            "<g transform=\"translate({px},{py})\">\n",
            px = p_summary.0,
            py = p_summary.1
        ));
        s.push_str(&format!(
            "<rect class=\"panel\" width=\"{}\" height=\"{}\" />\n",
            pw, ph
        ));
        s.push_str(&format!(
            "<text class=\"title\" x=\"20\" y=\"38\">{}</text>\n",
            xml_escape(&label)
        ));
        let summary_lines = [
            format!("samples       : {}", self.samples.len()),
            format!("duration      : {:.2} s", last.t),
            format!("max altitude  : {:.1} m", max_alt),
            format!("impact        : ({:.1}, {:.1}, {:.1}) m", last.pos.x, last.pos.y, last.pos.z),
            format!("ground range  : {:.1} m", ground_range),
            String::new(),
            "legend:".to_string(),
        ];
        for (i, line) in summary_lines.iter().enumerate() {
            let class = if line.starts_with("legend") {
                "summary-dim"
            } else {
                "summary"
            };
            s.push_str(&format!(
                "<text class=\"{}\" x=\"20\" y=\"{}\">{}</text>\n",
                class,
                70.0 + i as f64 * 22.0,
                xml_escape(line)
            ));
        }
        // Legend swatches.
        let legend_y = 70.0 + summary_lines.len() as f64 * 22.0;
        s.push_str(&format!(
            "<line class=\"boost\" x1=\"30\" y1=\"{:.1}\" x2=\"60\" y2=\"{:.1}\" />\n\
             <text class=\"summary-dim\" x=\"70\" y=\"{:.1}\">boost (thrust on)</text>\n",
            legend_y, legend_y, legend_y + 4.0
        ));
        s.push_str(&format!(
            "<line class=\"coast\" x1=\"30\" y1=\"{:.1}\" x2=\"60\" y2=\"{:.1}\" />\n\
             <text class=\"summary-dim\" x=\"70\" y=\"{:.1}\">coast (ballistic)</text>\n",
            legend_y + 22.0, legend_y + 22.0, legend_y + 26.0
        ));
        s.push_str(&format!(
            "<circle class=\"impact\" cx=\"45\" cy=\"{:.1}\" r=\"5\" />\n\
             <text class=\"summary-dim\" x=\"70\" y=\"{:.1}\">impact</text>\n",
            legend_y + 44.0, legend_y + 48.0
        ));
        s.push_str("</g>\n");

        s.push_str("</svg>\n");

        std::fs::write(path, s)
    }
}

fn bounds<I: Iterator<Item = f64>>(it: I) -> (f64, f64) {
    let mut lo = f64::INFINITY;
    let mut hi = f64::NEG_INFINITY;
    for v in it {
        if v < lo { lo = v; }
        if v > hi { hi = v; }
    }
    (lo, hi)
}

/// Round `x` up to a "nice" value of the form k * 10^n where
/// k ∈ {1, 1.5, 2, 2.5, 3, 4, 5, 7.5, 10}. Used for axis-span snapping.
fn nice_ceil(x: f64) -> f64 {
    if x <= 0.0 { return 1.0; }
    let exp = x.log10().floor();
    let base = 10f64.powf(exp);
    let m = x / base;
    let nice = if m <= 1.0 { 1.0 }
        else if m <= 1.5 { 1.5 }
        else if m <= 2.0 { 2.0 }
        else if m <= 2.5 { 2.5 }
        else if m <= 3.0 { 3.0 }
        else if m <= 4.0 { 4.0 }
        else if m <= 5.0 { 5.0 }
        else if m <= 7.5 { 7.5 }
        else { 10.0 };
    nice * base
}


fn xml_escape<S: AsRef<str>>(s: S) -> String {
    s.as_ref()
        .replace('&', "&amp;")
        .replace('<', "&lt;")
        .replace('>', "&gt;")
}
