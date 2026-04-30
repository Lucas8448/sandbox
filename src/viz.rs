//! Tiny terminal "retro scope" for visualizing the simulation.
//!
//! Two projection modes:
//!
//! * [`Projection::Ortho2D`] — flat XY plot, Y grows upward (physics
//!   convention, not screen convention).
//! * [`Projection::Perspective`] — true 3D camera with depth buffer and
//!   optional ground grid on the y=0 plane.
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
        Self {
            x,
            y,
            z: 0.0,
            glyph,
            color: Color::White,
        }
    }

    /// 3D constructor.
    pub fn new3(pos: Vec3, glyph: char) -> Self {
        Self {
            x: pos.x,
            y: pos.y,
            z: pos.z,
            glyph,
            color: Color::White,
        }
    }

    pub fn with_color(mut self, color: Color) -> Self {
        self.color = color;
        self
    }

    fn pos(&self) -> Vec3 {
        Vec3::new(self.x, self.y, self.z)
    }
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
    /// (~2:1), so set this to `2.0` to keep circles round-ish. Default 2.0.
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

    /// Build an orthonormal view basis (right, up, forward). `forward`
    /// points *from eye toward target* (positive depth in front).
    fn basis(&self) -> (Vec3, Vec3, Vec3) {
        let forward = (self.target - self.eye).normalize();
        let right = forward.cross(self.up).normalize();
        let up = right.cross(forward); // already unit since right & forward are
        (right, up, forward)
    }
}

/// Optional faint reference grid drawn on the world's y=0 plane (3D only).
#[derive(Debug, Clone, Copy)]
pub struct GroundGrid {
    pub half_extent: f64,
    pub step: f64,
    pub color: Color,
}

impl GroundGrid {
    pub fn new(half_extent: f64, step: f64) -> Self {
        Self {
            half_extent,
            step,
            color: Color::DarkYellow,
        }
    }
}

/// How world coordinates map onto the character grid.
#[derive(Debug, Clone, Copy)]
pub enum Projection {
    /// Flat XY plot. Z is ignored.
    Ortho2D {
        x_range: (f64, f64),
        y_range: (f64, f64),
    },
    /// True perspective 3D projection.
    Perspective { camera: Camera },
}

/// ASCII oscilloscope-style plotter. 2D or 3D depending on [`Projection`].
pub struct AsciiScope {
    pub width: u16,
    pub height: u16,
    pub projection: Projection,
    /// If true, previously drawn markers persist as faint trails.
    pub trails: bool,
    /// Ground reference. In 2D this draws a `=` line at y=0 if it lies in
    /// view. In 3D it samples a grid of dots on the y=0 plane.
    pub ground: Option<GroundGrid>,
    /// In 2D ortho a single character `=` line. Toggle separately from the
    /// 3D `ground` grid for back-compat with the old constructor.
    draw_ground_line_2d: bool,
    cells: Vec<Cell>,
    depth: Vec<f64>,
}

#[derive(Clone, Copy)]
struct Cell {
    glyph: char,
    color: Color,
}

impl Cell {
    const EMPTY: Cell = Cell {
        glyph: ' ',
        color: Color::Reset,
    };
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

    /// Toggle the simple 2D ground line (only meaningful for `Ortho2D`).
    pub fn with_ground(mut self, draw_ground: bool) -> Self {
        self.draw_ground_line_2d = draw_ground;
        self
    }

    /// Attach a 3D ground grid (only meaningful for `Perspective`).
    pub fn with_ground_grid(mut self, grid: GroundGrid) -> Self {
        self.ground = Some(grid);
        self
    }

    /// Update the camera (only meaningful for `Perspective`). Useful for
    /// orbiting / following shots.
    pub fn set_camera(&mut self, camera: Camera) {
        if let Projection::Perspective { camera: c } = &mut self.projection {
            *c = camera;
        }
    }

    fn idx(&self, col: u16, row: u16) -> usize {
        (row as usize) * (self.width as usize) + (col as usize)
    }

    /// Project a world point to (col, row, depth). Depth is meaningless in
    /// 2D (returned as 0.0). Returns `None` if culled / out of view.
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
                let nx = (rx / depth) * focal_x; // [-1,1] inside frustum
                let ny = (ry / depth) * focal_y;
                if !(-1.0..=1.0).contains(&nx) || !(-1.0..=1.0).contains(&ny) {
                    return None;
                }
                let col = ((nx * 0.5 + 0.5) * (self.width as f64 - 1.0)).round() as i32;
                // invert ny so +Y world is up on screen
                let row = (((-ny) * 0.5 + 0.5) * (self.height as f64 - 1.0)).round() as i32;
                Some((
                    col.clamp(0, self.width as i32 - 1) as u16,
                    row.clamp(0, self.height as i32 - 1) as u16,
                    depth,
                ))
            }
        }
    }

    /// Wipe the frame and depth buffer.
    pub fn clear(&mut self) {
        for c in &mut self.cells {
            *c = Cell::EMPTY;
        }
        for d in &mut self.depth {
            *d = f64::INFINITY;
        }
    }

    /// Fade existing glyphs to '.' (trail effect) and reset depth so new
    /// frames write freely on top of trails.
    fn fade_trails(&mut self) {
        for c in &mut self.cells {
            if c.glyph != ' ' && c.glyph != '.' && c.glyph != '`' {
                c.glyph = '.';
                c.color = Color::DarkGrey;
            } else if c.glyph == '.' {
                // second-stage fade
                c.glyph = '`';
            } else if c.glyph == '`' {
                c.glyph = ' ';
                c.color = Color::Reset;
            }
        }
        for d in &mut self.depth {
            *d = f64::INFINITY;
        }
    }

    /// Plot the cell at (col,row) if `depth` beats whatever's already there.
    fn put(&mut self, col: u16, row: u16, depth: f64, glyph: char, color: Color) {
        let i = self.idx(col, row);
        if depth <= self.depth[i] {
            self.cells[i] = Cell { glyph, color };
            self.depth[i] = depth;
        }
    }

    fn draw_ground(&mut self) {
        match self.projection {
            Projection::Ortho2D { x_range, .. } if self.draw_ground_line_2d => {
                if let Some((_, row, _)) = self.project(Vec3::new(x_range.0, 0.0, 0.0)) {
                    for col in 0..self.width {
                        self.put(col, row, f64::INFINITY, '=', Color::DarkYellow);
                    }
                }
            }
            Projection::Perspective { .. } => {
                if let Some(grid) = self.ground {
                    let h = grid.half_extent;
                    let mut x = -h;
                    while x <= h + 1e-9 {
                        let mut z = -h;
                        while z <= h + 1e-9 {
                            if let Some((col, row, depth)) =
                                self.project(Vec3::new(x, 0.0, z))
                            {
                                self.put(col, row, depth, '.', grid.color);
                            }
                            z += grid.step;
                        }
                        x += grid.step;
                    }
                }
            }
            _ => {}
        }
    }

    /// Render one frame.
    pub fn draw(&mut self, markers: &[Marker]) -> std::io::Result<()> {
        if self.trails {
            self.fade_trails();
        } else {
            self.clear();
        }

        self.draw_ground();

        // Sort markers far-to-near so closer ones still win via z-test, but
        // the depth buffer makes ordering moot. Kept simple.
        for m in markers {
            if let Some((col, row, depth)) = self.project(m.pos()) {
                self.put(col, row, depth, m.glyph, m.color);
            }
        }

        self.flush()
    }

    fn flush(&self) -> std::io::Result<()> {
        let mut out = stdout();
        out.queue(cursor::MoveTo(0, 0))?;

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
        out.flush()
    }

    /// Print a one-line HUD below the scope. Caller decides what to show.
    pub fn hud(&self, line: &str) -> std::io::Result<()> {
        let mut out = stdout();
        out.queue(Clear(ClearType::CurrentLine))?;
        out.queue(Print(line))?;
        out.queue(Print("\n"))?;
        out.flush()
    }
}

/// Convenience: clear screen, home cursor, hide cursor. Call once before the
/// animation loop so subsequent `draw` calls overwrite cleanly.
pub fn prepare_terminal() -> std::io::Result<()> {
    let mut out = stdout();
    out.queue(Clear(ClearType::All))?;
    out.queue(cursor::MoveTo(0, 0))?;
    out.queue(cursor::Hide)?;
    out.flush()
}

/// Restore cursor visibility. Call once after the animation loop.
pub fn restore_terminal() -> std::io::Result<()> {
    let mut out = stdout();
    out.queue(cursor::Show)?;
    out.queue(ResetColor)?;
    out.flush()
}
