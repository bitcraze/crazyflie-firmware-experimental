use std::num::NonZeroU32;
use std::rc::Rc;

use glow::HasContext;

use crate::config::{ActiveArea, BaseStation};

macro_rules! define_scoped_binding {
    (struct $binding_ty_name:ident => $obj_name:path, $param_name:path, $binding_fn:ident, $target_name:path) => {
        struct $binding_ty_name {
            saved_value: Option<$obj_name>,
            gl: Rc<glow::Context>,
        }

        impl $binding_ty_name {
            unsafe fn new(gl: &Rc<glow::Context>, new_binding: Option<$obj_name>) -> Self {
                unsafe {
                    let saved_value =
                        NonZeroU32::new(gl.get_parameter_i32($param_name) as u32).map($obj_name);
                    gl.$binding_fn($target_name, new_binding);
                    Self { saved_value, gl: gl.clone() }
                }
            }
        }

        impl Drop for $binding_ty_name {
            fn drop(&mut self) {
                unsafe {
                    self.gl.$binding_fn($target_name, self.saved_value);
                }
            }
        }
    };
    (struct $binding_ty_name:ident => $obj_name:path, $param_name:path, $binding_fn:ident) => {
        struct $binding_ty_name {
            saved_value: Option<$obj_name>,
            gl: Rc<glow::Context>,
        }

        impl $binding_ty_name {
            unsafe fn new(gl: &Rc<glow::Context>, new_binding: Option<$obj_name>) -> Self {
                unsafe {
                    let saved_value =
                        NonZeroU32::new(gl.get_parameter_i32($param_name) as u32).map($obj_name);
                    gl.$binding_fn(new_binding);
                    Self { saved_value, gl: gl.clone() }
                }
            }
        }

        impl Drop for $binding_ty_name {
            fn drop(&mut self) {
                unsafe {
                    self.gl.$binding_fn(self.saved_value);
                }
            }
        }
    };
}

define_scoped_binding!(struct ScopedTextureBinding => glow::NativeTexture, glow::TEXTURE_BINDING_2D, bind_texture, glow::TEXTURE_2D);
define_scoped_binding!(struct ScopedFrameBufferBinding => glow::NativeFramebuffer, glow::DRAW_FRAMEBUFFER_BINDING, bind_framebuffer, glow::DRAW_FRAMEBUFFER);
define_scoped_binding!(struct ScopedVBOBinding => glow::NativeBuffer, glow::ARRAY_BUFFER_BINDING, bind_buffer, glow::ARRAY_BUFFER);
define_scoped_binding!(struct ScopedVAOBinding => glow::NativeVertexArray, glow::VERTEX_ARRAY_BINDING, bind_vertex_array);

struct RenderTexture {
    texture: glow::Texture,
    width: u32,
    height: u32,
    fbo: glow::Framebuffer,
    gl: Rc<glow::Context>,
}

impl RenderTexture {
    unsafe fn new(gl: &Rc<glow::Context>, width: u32, height: u32) -> Self {
        unsafe {
            let fbo = gl.create_framebuffer().expect("Unable to create framebuffer");
            let texture = gl.create_texture().expect("Unable to allocate texture");

            let _saved_texture = ScopedTextureBinding::new(gl, Some(texture));

            let old_unpack_alignment = gl.get_parameter_i32(glow::UNPACK_ALIGNMENT);
            let old_unpack_row_length = gl.get_parameter_i32(glow::UNPACK_ROW_LENGTH);
            let old_unpack_skip_pixels = gl.get_parameter_i32(glow::UNPACK_SKIP_PIXELS);
            let old_unpack_skip_rows = gl.get_parameter_i32(glow::UNPACK_SKIP_ROWS);

            gl.pixel_store_i32(glow::UNPACK_ALIGNMENT, 1);
            gl.tex_parameter_i32(glow::TEXTURE_2D, glow::TEXTURE_MIN_FILTER, glow::LINEAR as i32);
            gl.tex_parameter_i32(glow::TEXTURE_2D, glow::TEXTURE_MAG_FILTER, glow::LINEAR as i32);
            gl.tex_parameter_i32(glow::TEXTURE_2D, glow::TEXTURE_WRAP_S, glow::CLAMP_TO_EDGE as i32);
            gl.tex_parameter_i32(glow::TEXTURE_2D, glow::TEXTURE_WRAP_T, glow::CLAMP_TO_EDGE as i32);
            gl.pixel_store_i32(glow::UNPACK_ROW_LENGTH, width as i32);
            gl.pixel_store_i32(glow::UNPACK_SKIP_PIXELS, 0);
            gl.pixel_store_i32(glow::UNPACK_SKIP_ROWS, 0);

            gl.tex_image_2d(
                glow::TEXTURE_2D, 0, glow::RGBA as _, width as _, height as _, 0,
                glow::RGBA as _, glow::UNSIGNED_BYTE as _, glow::PixelUnpackData::Slice(None),
            );

            let _saved_fbo = ScopedFrameBufferBinding::new(gl, Some(fbo));
            gl.framebuffer_texture_2d(
                glow::FRAMEBUFFER, glow::COLOR_ATTACHMENT0, glow::TEXTURE_2D, Some(texture), 0,
            );
            debug_assert_eq!(gl.check_framebuffer_status(glow::FRAMEBUFFER), glow::FRAMEBUFFER_COMPLETE);

            gl.pixel_store_i32(glow::UNPACK_ALIGNMENT, old_unpack_alignment);
            gl.pixel_store_i32(glow::UNPACK_ROW_LENGTH, old_unpack_row_length);
            gl.pixel_store_i32(glow::UNPACK_SKIP_PIXELS, old_unpack_skip_pixels);
            gl.pixel_store_i32(glow::UNPACK_SKIP_ROWS, old_unpack_skip_rows);

            Self { texture, width, height, fbo, gl: gl.clone() }
        }
    }
}

impl Drop for RenderTexture {
    fn drop(&mut self) {
        unsafe {
            self.gl.delete_framebuffer(self.fbo);
            self.gl.delete_texture(self.texture);
        }
    }
}

#[derive(Clone, Copy)]
pub struct UnitPos {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub color: [f32; 3],
    pub highlighted: bool,
    pub goto: Option<[f32; 3]>,
}

#[derive(Clone)]
pub struct TrailSegment {
    pub points: Vec<[f32; 3]>,
    pub color: [f32; 3],
}

#[derive(Clone, Copy)]
pub struct TakeoffMarker {
    pub x: f32,
    pub y: f32,
    pub color: [f32; 3],
}

#[derive(Clone, Copy)]
pub struct WandViz {
    pub pos: [f32; 3],
    pub dir: [f32; 3],
    pub ray_len: f32,
}

pub struct Scene3DRenderer {
    gl: Rc<glow::Context>,
    program: glow::Program,
    vbo: glow::Buffer,
    vao: glow::VertexArray,
    u_mvp: glow::UniformLocation,
    u_color: glow::UniformLocation,
    u_point_size: glow::UniformLocation,
    u_alpha: glow::UniformLocation,
    u_eye: glow::UniformLocation,
    u_fog_enabled: glow::UniformLocation,
    displayed_texture: RenderTexture,
    next_texture: RenderTexture,
}

const VERTEX_SHADER: &str = r#"#version 100
attribute vec3 position;
uniform mat4 u_mvp;
uniform float u_point_size;
uniform vec3 u_eye;
varying float v_dist;
void main() {
    gl_Position = u_mvp * vec4(position, 1.0);
    gl_PointSize = u_point_size;
    v_dist = length(position - u_eye);
}
"#;

const FRAGMENT_SHADER: &str = r#"#version 100
precision mediump float;
uniform vec3 u_color;
uniform float u_alpha;
uniform float u_fog_enabled;
varying float v_dist;
const vec3 FOG_COLOR = vec3(0.12, 0.12, 0.15);
const float FOG_START = 1.0;
const float FOG_END = 10.0;
void main() {
    float fog = u_fog_enabled * clamp((v_dist - FOG_START) / (FOG_END - FOG_START), 0.0, 1.0);
    vec3 col = mix(u_color, FOG_COLOR, fog);
    float a = mix(u_alpha, u_alpha * 0.3, fog);
    gl_FragColor = vec4(col, a);
}
"#;

impl Scene3DRenderer {
    pub fn new(gl: glow::Context) -> Self {
        let gl = Rc::new(gl);
        unsafe {
            let program = gl.create_program().expect("Cannot create program");

            let shaders_src = [
                (glow::VERTEX_SHADER, VERTEX_SHADER),
                (glow::FRAGMENT_SHADER, FRAGMENT_SHADER),
            ];

            let mut shaders = Vec::new();
            for (shader_type, src) in &shaders_src {
                let shader = gl.create_shader(*shader_type).expect("Cannot create shader");
                gl.shader_source(shader, src);
                gl.compile_shader(shader);
                if !gl.get_shader_compile_status(shader) {
                    panic!("Shader compile error: {}", gl.get_shader_info_log(shader));
                }
                gl.attach_shader(program, shader);
                shaders.push(shader);
            }

            gl.link_program(program);
            if !gl.get_program_link_status(program) {
                panic!("Program link error: {}", gl.get_program_info_log(program));
            }
            for s in shaders {
                gl.detach_shader(program, s);
                gl.delete_shader(s);
            }

            let u_mvp = gl.get_uniform_location(program, "u_mvp").unwrap();
            let u_color = gl.get_uniform_location(program, "u_color").unwrap();
            let u_point_size = gl.get_uniform_location(program, "u_point_size").unwrap();
            let u_alpha = gl.get_uniform_location(program, "u_alpha").unwrap();
            let u_eye = gl.get_uniform_location(program, "u_eye").unwrap();
            let u_fog_enabled = gl.get_uniform_location(program, "u_fog_enabled").unwrap();

            let vbo = gl.create_buffer().expect("Cannot create buffer");
            let vao = gl.create_vertex_array().expect("Cannot create VAO");

            let pos_loc = gl.get_attrib_location(program, "position").unwrap();
            gl.bind_vertex_array(Some(vao));
            gl.bind_buffer(glow::ARRAY_BUFFER, Some(vbo));
            gl.enable_vertex_attrib_array(pos_loc);
            gl.vertex_attrib_pointer_f32(pos_loc, 3, glow::FLOAT, false, 12, 0);
            gl.bind_buffer(glow::ARRAY_BUFFER, None);
            gl.bind_vertex_array(None);

            let displayed_texture = RenderTexture::new(&gl, 800, 600);
            let next_texture = RenderTexture::new(&gl, 800, 600);

            Self {
                gl, program, vbo, vao, u_mvp, u_color, u_point_size, u_alpha, u_eye, u_fog_enabled,
                displayed_texture, next_texture,
            }
        }
    }

    pub fn render(
        &mut self,
        width: u32,
        height: u32,
        yaw: f32,
        pitch: f32,
        distance: f32,
        pan_x: f32,
        pan_y: f32,
        units: &[UnitPos],
        trails: &[TrailSegment],
        takeoff_markers: &[TakeoffMarker],
        goto_history: &[([f32; 3], [f32; 3])],
        active_area: Option<ActiveArea>,
        base_stations: &[BaseStation],
        show_axes: bool,
        show_grid: bool,
        show_fog: bool,
        wand: Option<&WandViz>,
    ) -> slint::Image {
        let width = width.max(1);
        let height = height.max(1);

        unsafe {
            let gl = &self.gl;

            if self.next_texture.width != width || self.next_texture.height != height {
                let mut new_tex = RenderTexture::new(gl, width, height);
                std::mem::swap(&mut self.next_texture, &mut new_tex);
            }

            let _saved_fbo = ScopedFrameBufferBinding::new(gl, Some(self.next_texture.fbo));
            let mut saved_viewport = [0i32; 4];
            gl.get_parameter_i32_slice(glow::VIEWPORT, &mut saved_viewport);
            gl.viewport(0, 0, width as _, height as _);

            gl.clear_color(0.12, 0.12, 0.15, 1.0);
            gl.clear(glow::COLOR_BUFFER_BIT | glow::DEPTH_BUFFER_BIT);
            gl.enable(glow::DEPTH_TEST);

            gl.use_program(Some(self.program));

            let _saved_vao = ScopedVAOBinding::new(gl, Some(self.vao));
            let _saved_vbo = ScopedVBOBinding::new(gl, Some(self.vbo));

            let aspect = width as f32 / height as f32;
            let mvp = build_mvp(yaw, pitch, distance, pan_x, pan_y, aspect);
            gl.uniform_matrix_4_f32_slice(Some(&self.u_mvp), false, &mvp);
            gl.uniform_1_f32(Some(&self.u_point_size), 1.0);
            gl.uniform_1_f32(Some(&self.u_alpha), 1.0);

            // Fog: pass camera eye position and toggle
            let eye_x = distance * pitch.cos() * yaw.cos() + pan_x;
            let eye_y = distance * pitch.cos() * yaw.sin() + pan_y;
            let eye_z = distance * pitch.sin();
            gl.uniform_3_f32(Some(&self.u_eye), eye_x, eye_y, eye_z);
            gl.uniform_1_f32(Some(&self.u_fog_enabled), if show_fog { 1.0 } else { 0.0 });

            // Draw ground grid (clipped to active area if set)
            if show_grid {
                let (gx_min, gx_max, gy_min, gy_max) = if let Some(aa) = active_area {
                    (aa.min_x.floor() as i32, aa.max_x.ceil() as i32,
                     aa.min_y.floor() as i32, aa.max_y.ceil() as i32)
                } else {
                    (-5, 5, -5, 5)
                };
                let mut grid_verts = Vec::new();
                // Vertical lines (constant x, varying y)
                for i in gx_min..=gx_max {
                    let v = i as f32;
                    grid_verts.extend_from_slice(&[v, gy_min as f32, 0.0, v, gy_max as f32, 0.0]);
                }
                // Horizontal lines (constant y, varying x)
                for i in gy_min..=gy_max {
                    let v = i as f32;
                    grid_verts.extend_from_slice(&[gx_min as f32, v, 0.0, gx_max as f32, v, 0.0]);
                }
                gl.uniform_3_f32(Some(&self.u_color), 0.3, 0.3, 0.35);
                upload_and_draw(gl, self.vbo, &grid_verts);
                gl.draw_arrays(glow::LINES, 0, grid_verts.len() as i32 / 3);

                // Draw active area border
                if let Some(aa) = active_area {
                    let border = [
                        aa.min_x, aa.min_y, 0.0, aa.max_x, aa.min_y, 0.0,
                        aa.max_x, aa.min_y, 0.0, aa.max_x, aa.max_y, 0.0,
                        aa.max_x, aa.max_y, 0.0, aa.min_x, aa.max_y, 0.0,
                        aa.min_x, aa.max_y, 0.0, aa.min_x, aa.min_y, 0.0,
                    ];
                    gl.uniform_3_f32(Some(&self.u_color), 0.5, 0.5, 0.2);
                    upload_and_draw(gl, self.vbo, &border);
                    gl.draw_arrays(glow::LINES, 0, 8);
                }
            }

            // Draw axes (2m each)
            if show_axes {
                // X axis - red
                gl.uniform_3_f32(Some(&self.u_color), 0.94, 0.27, 0.27);
                upload_and_draw(gl, self.vbo, &[0.0, 0.0, 0.0, 2.0, 0.0, 0.0]);
                gl.draw_arrays(glow::LINES, 0, 2);

                // Y axis - green
                gl.uniform_3_f32(Some(&self.u_color), 0.29, 0.85, 0.50);
                upload_and_draw(gl, self.vbo, &[0.0, 0.0, 0.0, 0.0, 2.0, 0.0]);
                gl.draw_arrays(glow::LINES, 0, 2);

                // Z axis - blue
                gl.uniform_3_f32(Some(&self.u_color), 0.38, 0.65, 0.98);
                upload_and_draw(gl, self.vbo, &[0.0, 0.0, 0.0, 0.0, 0.0, 2.0]);
                gl.draw_arrays(glow::LINES, 0, 2);
            }

            // Draw lighthouse base stations
            for bs in base_stations {
                // Vertical line from floor to base station
                gl.uniform_3_f32(Some(&self.u_color), 0.6, 0.4, 0.8);
                gl.uniform_1_f32(Some(&self.u_point_size), 1.0);
                upload_and_draw(gl, self.vbo, &[
                    bs.origin[0], bs.origin[1], 0.0,
                    bs.origin[0], bs.origin[1], bs.origin[2],
                ]);
                gl.draw_arrays(glow::LINES, 0, 2);

                // Point at base station position
                gl.uniform_1_f32(Some(&self.u_point_size), 10.0);
                gl.uniform_3_f32(Some(&self.u_color), 0.7, 0.5, 1.0);
                upload_and_draw(gl, self.vbo, &[bs.origin[0], bs.origin[1], bs.origin[2]]);
                gl.draw_arrays(glow::POINTS, 0, 1);
            }

            // Draw takeoff position markers (circles on the ground plane)
            for marker in takeoff_markers {
                const CIRCLE_SEGMENTS: usize = 24;
                const RADIUS: f32 = 0.06;
                let mut circle_verts = Vec::with_capacity(CIRCLE_SEGMENTS * 6);
                for i in 0..CIRCLE_SEGMENTS {
                    let a0 = (i as f32 / CIRCLE_SEGMENTS as f32) * std::f32::consts::TAU;
                    let a1 = ((i + 1) as f32 / CIRCLE_SEGMENTS as f32) * std::f32::consts::TAU;
                    circle_verts.extend_from_slice(&[
                        marker.x + RADIUS * a0.cos(), marker.y + RADIUS * a0.sin(), 0.0,
                        marker.x + RADIUS * a1.cos(), marker.y + RADIUS * a1.sin(), 0.0,
                    ]);
                }
                gl.uniform_3_f32(Some(&self.u_color), marker.color[0], marker.color[1], marker.color[2]);
                gl.uniform_1_f32(Some(&self.u_alpha), 0.7);
                gl.enable(glow::BLEND);
                gl.blend_func(glow::SRC_ALPHA, glow::ONE_MINUS_SRC_ALPHA);
                upload_and_draw(gl, self.vbo, &circle_verts);
                gl.draw_arrays(glow::LINES, 0, circle_verts.len() as i32 / 3);
                gl.uniform_1_f32(Some(&self.u_alpha), 1.0);
                gl.disable(glow::BLEND);
            }

            // Disable depth test for copter overlays (trails, points, highlights)
            // so they always render on top of the grid/axes
            gl.disable(glow::DEPTH_TEST);

            // Draw trail lines for selected copter
            for trail in trails {
                if trail.points.len() >= 2 {
                    let total = trail.points.len();
                    let mut trail_verts = Vec::with_capacity(total * 3);
                    for p in &trail.points {
                        trail_verts.extend_from_slice(p);
                    }
                    gl.uniform_3_f32(
                        Some(&self.u_color),
                        trail.color[0] * 0.7,
                        trail.color[1] * 0.7,
                        trail.color[2] * 0.7,
                    );
                    gl.uniform_1_f32(Some(&self.u_point_size), 1.0);
                    upload_and_draw(gl, self.vbo, &trail_verts);
                    gl.draw_arrays(glow::LINE_STRIP, 0, total as i32);
                }
            }

            // Draw goto history points for selected copter
            if !goto_history.is_empty() {
                gl.uniform_1_f32(Some(&self.u_point_size), 6.0);
                gl.uniform_1_f32(Some(&self.u_alpha), 1.0);
                gl.uniform_3_f32(Some(&self.u_color), 1.0, 0.2, 0.2);
                for &(pos, _color) in goto_history {
                    upload_and_draw(gl, self.vbo, &pos);
                    gl.draw_arrays(glow::POINTS, 0, 1);
                }
            }

            // Draw units as points with drop lines
            for unit in units {
                // Vertical drop line to ground (draw first, behind the point)
                gl.uniform_3_f32(
                    Some(&self.u_color),
                    unit.color[0] * 0.4,
                    unit.color[1] * 0.4,
                    unit.color[2] * 0.4,
                );
                gl.uniform_1_f32(Some(&self.u_point_size), 1.0);
                upload_and_draw(gl, self.vbo, &[unit.x, unit.y, unit.z, unit.x, unit.y, 0.0]);
                gl.draw_arrays(glow::LINES, 0, 2);

                // White highlight ring behind the colored point
                if unit.highlighted {
                    gl.uniform_1_f32(Some(&self.u_point_size), 24.0);
                    gl.uniform_3_f32(Some(&self.u_color), 1.0, 1.0, 1.0);
                    upload_and_draw(gl, self.vbo, &[unit.x, unit.y, unit.z]);
                    gl.draw_arrays(glow::POINTS, 0, 1);
                }

                // Colored point on top
                let point_size = if unit.highlighted { 18.0 } else { 12.0 };
                gl.uniform_1_f32(Some(&self.u_point_size), point_size);
                gl.uniform_3_f32(Some(&self.u_color), unit.color[0], unit.color[1], unit.color[2]);
                upload_and_draw(gl, self.vbo, &[unit.x, unit.y, unit.z]);
                gl.draw_arrays(glow::POINTS, 0, 1);

                // Goto target: dotted semi-transparent line + small marker
                if let Some(goto) = unit.goto {
                    gl.enable(glow::BLEND);
                    gl.blend_func(glow::SRC_ALPHA, glow::ONE_MINUS_SRC_ALPHA);

                    // Build dashed line segments
                    let dash_len = 0.05_f32;
                    let dx = goto[0] - unit.x;
                    let dy = goto[1] - unit.y;
                    let dz = goto[2] - unit.z;
                    let total_len = (dx * dx + dy * dy + dz * dz).sqrt();
                    let num_dashes = (total_len / (dash_len * 2.0)).ceil() as usize;
                    let mut dash_verts = Vec::with_capacity(num_dashes * 6);
                    if total_len > 0.001 {
                        for i in 0..num_dashes {
                            let t0 = (i as f32 * 2.0 * dash_len) / total_len;
                            let t1 = ((i as f32 * 2.0 + 1.0) * dash_len / total_len).min(1.0);
                            if t0 >= 1.0 { break; }
                            dash_verts.extend_from_slice(&[
                                unit.x + dx * t0, unit.y + dy * t0, unit.z + dz * t0,
                                unit.x + dx * t1, unit.y + dy * t1, unit.z + dz * t1,
                            ]);
                        }
                    }

                    gl.uniform_3_f32(
                        Some(&self.u_color),
                        unit.color[0] * 0.7,
                        unit.color[1] * 0.7,
                        unit.color[2] * 0.7,
                    );
                    gl.uniform_1_f32(Some(&self.u_alpha), 0.4);
                    gl.uniform_1_f32(Some(&self.u_point_size), 1.0);
                    if !dash_verts.is_empty() {
                        upload_and_draw(gl, self.vbo, &dash_verts);
                        gl.draw_arrays(glow::LINES, 0, dash_verts.len() as i32 / 3);
                    }

                    // Small marker at goto position
                    gl.uniform_1_f32(Some(&self.u_point_size), 8.0);
                    gl.uniform_1_f32(Some(&self.u_alpha), 0.5);
                    gl.uniform_3_f32(
                        Some(&self.u_color),
                        unit.color[0] * 0.8,
                        unit.color[1] * 0.8,
                        unit.color[2] * 0.8,
                    );
                    upload_and_draw(gl, self.vbo, &[goto[0], goto[1], goto[2]]);
                    gl.draw_arrays(glow::POINTS, 0, 1);

                    gl.uniform_1_f32(Some(&self.u_alpha), 1.0);
                    gl.disable(glow::BLEND);
                }
            }

            // Draw wand ray (forward direction only)
            if let Some(w) = wand {
                let ray_len = w.ray_len;
                let p0 = w.pos;
                let p1 = [
                    w.pos[0] + w.dir[0] * ray_len,
                    w.pos[1] + w.dir[1] * ray_len,
                    w.pos[2] + w.dir[2] * ray_len,
                ];

                // Ray line - cyan
                gl.uniform_3_f32(Some(&self.u_color), 0.0, 0.9, 0.9);
                gl.uniform_1_f32(Some(&self.u_alpha), 0.7);
                gl.uniform_1_f32(Some(&self.u_point_size), 1.0);
                gl.enable(glow::BLEND);
                gl.blend_func(glow::SRC_ALPHA, glow::ONE_MINUS_SRC_ALPHA);
                upload_and_draw(gl, self.vbo, &[p0[0], p0[1], p0[2], p1[0], p1[1], p1[2]]);
                gl.draw_arrays(glow::LINES, 0, 2);

                // Point at wand position
                gl.uniform_1_f32(Some(&self.u_point_size), 14.0);
                gl.uniform_3_f32(Some(&self.u_color), 0.0, 1.0, 0.8);
                gl.uniform_1_f32(Some(&self.u_alpha), 0.9);
                upload_and_draw(gl, self.vbo, &[w.pos[0], w.pos[1], w.pos[2]]);
                gl.draw_arrays(glow::POINTS, 0, 1);

                gl.uniform_1_f32(Some(&self.u_alpha), 1.0);
                gl.disable(glow::BLEND);
            }

            gl.use_program(None);
            gl.disable(glow::DEPTH_TEST);
            gl.viewport(saved_viewport[0], saved_viewport[1], saved_viewport[2], saved_viewport[3]);
        }

        let result = unsafe {
            slint::BorrowedOpenGLTextureBuilder::new_gl_2d_rgba_texture(
                self.next_texture.texture.0,
                (self.next_texture.width, self.next_texture.height).into(),
            )
            .build()
        };

        std::mem::swap(&mut self.next_texture, &mut self.displayed_texture);
        result
    }
}

impl Drop for Scene3DRenderer {
    fn drop(&mut self) {
        unsafe {
            self.gl.delete_program(self.program);
            self.gl.delete_vertex_array(self.vao);
            self.gl.delete_buffer(self.vbo);
        }
    }
}

unsafe fn upload_and_draw(gl: &glow::Context, vbo: glow::Buffer, data: &[f32]) {
    unsafe {
        gl.bind_buffer(glow::ARRAY_BUFFER, Some(vbo));
        gl.buffer_data_u8_slice(
            glow::ARRAY_BUFFER,
            data.align_to().1,
            glow::DYNAMIC_DRAW,
        );
    }
}

pub fn project_to_screen(
    point: [f32; 3],
    mvp: &[f32; 16],
    width: u32,
    height: u32,
) -> Option<(f32, f32)> {
    let x = mvp[0] * point[0] + mvp[4] * point[1] + mvp[8]  * point[2] + mvp[12];
    let y = mvp[1] * point[0] + mvp[5] * point[1] + mvp[9]  * point[2] + mvp[13];
    let w = mvp[3] * point[0] + mvp[7] * point[1] + mvp[11] * point[2] + mvp[15];

    if w <= 0.0 {
        return None;
    }

    let ndc_x = x / w;
    let ndc_y = y / w;

    let screen_x = (ndc_x + 1.0) * 0.5 * width as f32;
    let screen_y = (ndc_y + 1.0) * 0.5 * height as f32;

    Some((screen_x, screen_y))
}

pub fn compute_mvp(yaw: f32, pitch: f32, distance: f32, pan_x: f32, pan_y: f32, aspect: f32) -> [f32; 16] {
    build_mvp(yaw, pitch, distance, pan_x, pan_y, aspect)
}

fn build_mvp(yaw: f32, pitch: f32, distance: f32, pan_x: f32, pan_y: f32, aspect: f32) -> [f32; 16] {
    let cam_x = distance * pitch.cos() * yaw.cos();
    let cam_y = distance * pitch.cos() * yaw.sin();
    let cam_z = distance * pitch.sin();

    let target = [pan_x, pan_y, 0.5];
    let eye = [cam_x + pan_x, cam_y + pan_y, cam_z];

    let view = look_at(eye, target, [0.0, 0.0, 1.0]);
    let proj = perspective(45.0_f32.to_radians(), aspect, 0.1, 100.0);

    mat4_mul(&proj, &view)
}

fn look_at(eye: [f32; 3], center: [f32; 3], up: [f32; 3]) -> [f32; 16] {
    let f = normalize(sub(center, eye));
    let s = normalize(cross(f, up));
    let u = cross(s, f);

    [
        s[0], u[0], -f[0], 0.0,
        s[1], u[1], -f[1], 0.0,
        s[2], u[2], -f[2], 0.0,
        -dot(s, eye), -dot(u, eye), dot(f, eye), 1.0,
    ]
}

fn perspective(fov: f32, aspect: f32, near: f32, far: f32) -> [f32; 16] {
    let f = 1.0 / (fov / 2.0).tan();
    let nf = 1.0 / (near - far);
    [
        f / aspect, 0.0, 0.0, 0.0,
        0.0, -f, 0.0, 0.0,
        0.0, 0.0, (far + near) * nf, -1.0,
        0.0, 0.0, 2.0 * far * near * nf, 0.0,
    ]
}

fn mat4_mul(a: &[f32; 16], b: &[f32; 16]) -> [f32; 16] {
    let mut out = [0.0f32; 16];
    for i in 0..4 {
        for j in 0..4 {
            for k in 0..4 {
                out[i * 4 + j] += a[k * 4 + j] * b[i * 4 + k];
            }
        }
    }
    out
}

fn sub(a: [f32; 3], b: [f32; 3]) -> [f32; 3] { [a[0]-b[0], a[1]-b[1], a[2]-b[2]] }
fn dot(a: [f32; 3], b: [f32; 3]) -> f32 { a[0]*b[0] + a[1]*b[1] + a[2]*b[2] }
fn cross(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]
}
fn normalize(v: [f32; 3]) -> [f32; 3] {
    let len = dot(v, v).sqrt();
    if len < 1e-10 { return [0.0; 3]; }
    [v[0]/len, v[1]/len, v[2]/len]
}
