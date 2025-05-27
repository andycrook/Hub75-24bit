
# HUB75 MicroPython LED Matrix Driver

**Author:** Andy Crook  
**Platform:** Raspberry Pi Pico 2W

**Target Display:** 64x64 HUB75 RGB LED matrix  
**License:** GPL-3  

---

## Overview

This is a MicroPython driver for HUB75-style RGB LED matrices (64x64). It features:

- Full 24-bit color support using 8-bit binary-coded modulation (BCM)
- Hardware-accelerated drawing with PIO and triple-buffering
- BMP loading (24/32-bit) with transparency and gamma correction
- BDF and custom font rendering
- 3D OBJ textured model support with per-face normals and keyframed animation
- support with Catmull rom spline interpolation
- Particle emission system (simple)
- Advanced blending modes (multiply, screen, overlay, etc.) for bmp and framebuffers
- BDF font, custom mfont format, text effects and marquee scrolling text

This driver leverages the Raspberry Pi Pico's PIO and multi-core features for performance.
One core sends data to the display with BCM timings and the other is free for all library
drawing operations.

---

## Contents

- [Class: `Hub75`](#class-hub75)
  - [Initialization](#initialization)
  - [Drawing Primitives](#drawing-primitives)
  - [Pixel I/O](#pixel-io)
  - [Frame Buffer Control](#frame-buffer-control)
  - [BMP Loading](#bmp-loading)
  - [Color and Blending Utilities](#color-and-blending-utilities)
  - [Font and Text Rendering](#font-and-text-rendering)
  - [Bitmap Buffering](#bitmap-buffering)
  - [Marquee and Scrolling](#marquee-and-scrolling)
  - [Geometry Utilities](#geometry-utilities)
  - [3D Object Support](#3d-object-support)
  - [Miscellaneous](#miscellaneous)

---

## Class: `Hub75`

### Initialization

```python
Hub75(
    data_pin_start=2,
    clock_pin=13,
    latch_pin_start=14,
    row_pin_start=8,
    num_rows=32,
    blocks_per_row=16
)
```

Initializes the HUB75 matrix controller. It sets up three PIO state machines:
- `led_data`: streams RGB bitplane data to the matrix
- `address_counter`: selects row address lines
- `output`: handles BCM timing and frame display

Also sets up gamma LUTs, pixel lookup tables, and starts a separate thread for continuous frame updates.

---



### Drawing Primitives

The following methods provide 2D drawing capabilities for the matrix, using RGB values (0–255 per channel).

#### `set_pixel(x, y, g, r, b, vx, vy, vw, vh)`
Draw a single pixel at `(x, y)` with RGB color. Clipped to viewport (`vx, vy, vw, vh`).
set_pixel is the core of most drawing operations. It's in viper for performance but this
means that it has to have the 4 ints for boundaries passed. This can be reworked with a
function calling this viper function for a simpler user function call, as the cost of a few
us of time.

#### `get_pixel(x, y) -> (r, g, b)`
Returns the RGB color of the pixel at `(x, y)` as a tuple.

#### `hline(x, y, length, r,g,b)`
Draws a horizontal line from `(x, y)` of given length and color.

#### `vline(x, y, length, r,g,b)`
Draws a vertical line from `(x, y)` of given length and color.

#### `line(x0, y0, x1, y1, r,g,b)`
Draws a line from `(x0, y0)` to `(x1, y1)` using Bresenham's algorithm.

#### `box(x, y, width, height, r,g,b, filled)`
Draws a rectangle. If `filled` is true, fills the box with color.

#### `ellipse(x, y, rx, ry, r,g,b, filled)`
Draws an ellipse centered at `(x, y)` with radii `rx`, `ry`. Fills it if `filled` is true.

#### `polygon(points, r,g,b, closed=True)`
Draws lines between each point in `points`. Closes the shape if `closed=True`.

---

### Pixel I/O

#### `fill(r=0, g=0, b=0)`
Fills the entire frame buffer with the specified color.

#### `clear()`
Clears all frame buffers (current and next) to black.

#### `refresh()`
Swaps the `frame_buffer_temp` with `frame_buffer_ready`, marking the new frame as ready to send.

---

### Frame Buffer Control

#### `save_frame(filename)`
Saves the current frame buffer as a `.bin` file (8 bitplanes, 2048 bytes each).

#### `load_frame(filename)`
Loads a saved frame buffer from disk into the working buffer.

#### `blend_frames(path1, path2, value=128, mode="fade")`
Blends two `.bin` frame files into the current buffer. Supports modes:
- `"fade"`: alpha blend
- `"add"`, `"multiply"`, `"screen"`, `"lighten"`, `"darken"`, `"overlay"`

#### `scroll_vertical(amount)`
Scrolls the display vertically by `amount` pixels (wraps around).

#### `scroll_horizontal(amount)`
Scrolls the display horizontally by `amount` pixels (wraps around).

---


### BMP Loading

#### `load_bmp(filename, x1=0, y1=0, gamma=2.2, brightness=1.0, contrast=1.0, buffered=0, scale=1, hue=0, return_data=0, blendmode="none")`
Loads a 24/32-bit BMP file to the matrix. Supports:
- Gamma, brightness, contrast adjustment
- Hue shift in degrees
- Buffered mode (stores as sprite)
- Return mode (returns raw RGB data)
- Blend modes: `"alpha"`, `"multiply"`, `"screen"`, `"lighten"`, `"darken"`, `"add"`
- Gamma of 2.2 is the default and recommended to make images look good on an LED matrix

#### `show_bmp(index, x, y)`
Draws a previously loaded buffered BMP at `(x, y)`
This works with buffer mode, and the pixel data is internally held so that it can be redrawn
at arbitrary locations without having to load in a bmp.

#### `count_bmp() -> (count, [lengths])`
Returns number of buffered BMPs in memory and pixel counts.

#### `erase_bmp(index)`
Removes a buffered BMP from memory from the specified index

---

### Color and Blending Utilities

#### `adjust_gamma(value, gamma)`
Applies gamma correction to an 8-bit color value.

#### `adjust_brightness(value, brightness)`
Applies brightness adjustment to an 8-bit color value.

#### `adjust_contrast(value, contrast)`
Applies contrast adjustment to an 8-bit color value.

#### `blend_rgb888_pixel(R0, G0, B0, R1, G1, B1, A1, mode)`
Blends two 24-bit RGB pixels using alpha and selected blend mode.

#### `hue_shift_rgb888(R, G, B, degrees)`
Applies a hue rotation to an RGB color.

---

### Font and Text Rendering

#### `load_bdf_font(path, code_range=(32,127))`
Loads a BDF font (expensive, not recommended for full fonts).

#### `save_minifont(path)`
Saves current font to a compact `.mfont` format (binary).
This is the preferred font format. External py mfont editor available
to edit mfont glyphs directly.

#### `load_minifont(path)`
Loads a `.mfont` file.

#### `monospace_digits(font)`
Forces all digits in a font to have the same width and x-advance.

#### `draw_char(x, y, char, color, BGcolor, background_mode=0, buffer=0)`
Draws a character from the loaded font. Helper for draw_text.

#### `draw_text(x, y, text, color, BGcolor, background_mode=0, buffer=2, shadow=0, marquee=False)`
Draws a string using the loaded font. Supports:
- Background fill modes
- Shadow direction
- Marquee support (repeats text)
- For speed, buffer=0 and the function will draw to the screen directly. With buffer mode,
- the text is held in a buffer and further pixel operations can be performed, like
- background surround, shadow effects, and marquee drawing.

#### `draw_text_buffer(x, y, color, BGcolor, shadow=0)`
Renders the text from the pixel buffer with shadow or background.

---
