//! Tiny terminal "retro scope" for visualizing 2D projections of the simulation.
//!
//! Renders points onto a character grid (X horizontal, Y vertical, Y growing
//! upward like in physics — not like in screen coordinates). Designed for
//! examples and quick debugging; no allocation in the hot path beyond the
//! frame buffer.
//!
//! Enabled with the `viz` feature.

use std::io::{Write, stdout};

use crossterm::{
    QueueableCommand, cursor,
    style::{Color, Print, ResetColor, SetForegroundColor},
    terminal::{Clear, ClearType},
};

/// A single body to draw on the scope.
#[derive(Debug, Clone, Copy)]
pub struct Marker {
    pub x: f64,
    pub y: f64,
    pub glyph: char,
    pub color: Color,
}

impl Marker {
    pub fn new(x: f64, y: f64, glyph: char) -> Self {
        Self {
            x,
            y,
            glyph,
            color: Color::White,
        }
    }

    pub fn with_color(mut self, color: Color) -> Self {
        self.color = color;
        self
    }
}

/// ASCII oscilloscope-style 2D plotter.
pub struct AsciiScope {
    pub width: u16,
    pub height: u16,
    pub x_range: (f64, f64),
    pub y_range: (f64, f64),
    /// If true, persist previously drawn markers as faint trails.
    pub trails: bool,
    /// Background grid character used as ground reference at y=0.
    pub draw_ground: bool,
    cells: Vec<Cell>,
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
    pub fn new(width: u16, height: u16, x_range: (f64, f64), y_range: (f64, f64)) -> Self {
        Self {
            width,
            height,
            x_range,
            y_range,
            trails: true,
            draw_ground: true,
            cells: vec![Cell::EMPTY; (width as usize) * (height as usize)],
        }
    }

    pub fn with_trails(mut self, trails: bool) -> Self {
        self.trails = trails;
        self
    }

    pub fn with_ground(mut self, draw_ground: bool) -> Self {
        self.draw_ground = draw_ground;
        self
    }

    fn idx(&self, col: u16, row: u16) -> usize {
        (row as usize) * (self.width as usize) + (col as usize)
    }

    fn project(&self, x: f64, y: f64) -> Option<(u16, u16)> {
        let (xmin, xmax) = self.x_range;
        let (ymin, ymax) = self.y_range;
        if x < xmin || x > xmax || y < ymin || y > ymax {
            return None;
        }
        let nx = (x - xmin) / (xmax - xmin);
        let ny = (y - ymin) / (ymax - ymin);
        let col = (nx * (self.width as f64 - 1.0)).round() as i32;
        // invert Y: row 0 is top of terminal but top of physics range.
        let row = ((1.0 - ny) * (self.height as f64 - 1.0)).round() as i32;
        Some((col.clamp(0, self.width as i32 - 1) as u16,
              row.clamp(0, self.height as i32 - 1) as u16))
    }

    /// Wipe the frame (ignoring trails setting — full reset).
    pub fn clear(&mut self) {
        for c in &mut self.cells {
            *c = Cell::EMPTY;
        }
    }

    /// Fade trails: dot any non-empty cell to '.' so live markers stand out.
    fn fade_trails(&mut self) {
        for c in &mut self.cells {
            if c.glyph != ' ' {
                c.glyph = '.';
                c.color = Color::DarkGrey;
            }
        }
    }

    /// Plot one frame from the given markers and render to stdout.
    pub fn draw(&mut self, markers: &[Marker]) -> std::io::Result<()> {
        if self.trails {
            self.fade_trails();
        } else {
            self.clear();
        }

        if self.draw_ground {
            // ground line at world y = 0 if it lies inside y_range
            if let Some((_, row)) = self.project(self.x_range.0, 0.0) {
                for col in 0..self.width {
                    let i = self.idx(col, row);
                    self.cells[i] = Cell {
                        glyph: '=',
                        color: Color::DarkYellow,
                    };
                }
            }
        }

        for m in markers {
            if let Some((col, row)) = self.project(m.x, m.y) {
                let i = self.idx(col, row);
                self.cells[i] = Cell {
                    glyph: m.glyph,
                    color: m.color,
                };
            }
        }

        self.flush()
    }

    fn flush(&self) -> std::io::Result<()> {
        let mut out = stdout();
        out.queue(cursor::MoveTo(0, 0))?;

        // top border
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

/// Convenience: clear the entire screen and home the cursor. Call once before
/// the animation loop so subsequent `draw` calls overwrite cleanly.
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
