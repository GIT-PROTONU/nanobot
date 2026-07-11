"""GPU-accelerated webcam vision (Mali-450/GLES2 via `lima`): continuous YUYV capture,
on-GPU motion-diff ("PIR") + colour-threshold blob tracking, and a JPEG-encoded live-view
tee for the browser. See CLAUDE.md / memory `gpu-vision-*` for the full design writeup —
this implements the "flip camera ownership" architecture: GpuVision is the sole, continuous
camera owner; the browser's live view is a downstream tee off the same captured frames, not
a second V4L2 session.

Zero extra Python dependencies — raw ctypes against libEGL/libGLESv2 (same philosophy as
`mjpeg_camera.py`'s raw V4L2 ioctls and `scripts/gpu_vision_spike.py`'s EGL/GLES probe, which
this reuses the proven context-creation pattern from).
"""
import ctypes
import ctypes.util
import os
import threading
import time

from . import mjpeg_camera

os.environ.setdefault("EGL_PLATFORM", "surfaceless")

# ---- EGL / GLES2 constants ----
EGL_DEFAULT_DISPLAY = 0
EGL_NO_CONTEXT = 0
EGL_SURFACE_TYPE = 0x3033
EGL_PBUFFER_BIT = 0x0001
EGL_RENDERABLE_TYPE = 0x3040
EGL_OPENGL_ES2_BIT = 0x0004
EGL_RED_SIZE = 0x3024
EGL_GREEN_SIZE = 0x3023
EGL_BLUE_SIZE = 0x3022
EGL_ALPHA_SIZE = 0x3021
EGL_NONE = 0x3038
EGL_WIDTH = 0x3057
EGL_HEIGHT = 0x3056
EGL_CONTEXT_CLIENT_VERSION = 0x3098
EGL_OPENGL_ES_API = 0x30A0

GL_FRAGMENT_SHADER = 0x8B30
GL_VERTEX_SHADER = 0x8B31
GL_COMPILE_STATUS = 0x8B81
GL_LINK_STATUS = 0x8B82
GL_FRAMEBUFFER = 0x8D40
GL_COLOR_ATTACHMENT0 = 0x8CE0
GL_TEXTURE_2D = 0x0DE1
GL_RGBA = 0x1908
GL_LUMINANCE_ALPHA = 0x190A
GL_UNSIGNED_BYTE = 0x1401
GL_FRAMEBUFFER_COMPLETE = 0x8CD5
GL_TRIANGLE_STRIP = 0x0005
GL_FLOAT = 0x1406
GL_TEXTURE_MIN_FILTER = 0x2801
GL_TEXTURE_MAG_FILTER = 0x2800
GL_NEAREST = 0x2600
GL_LINEAR = 0x2601
GL_TEXTURE_WRAP_S = 0x2802
GL_TEXTURE_WRAP_T = 0x2803
GL_CLAMP_TO_EDGE = 0x812F
GL_TEXTURE0 = 0x84C0
GL_UNPACK_ALIGNMENT = 0x0CF5


def _load(name):
    path = ctypes.util.find_library(name) or f"lib{name}.so.1"
    return ctypes.CDLL(path)


class _GL:
    """Thin ctypes binding + a couple of small helpers (compile/link, FBO+texture).
    One instance per EGL context (GL state/objects are context-scoped)."""

    def __init__(self):
        self.egl = _load("EGL")
        self.gl = _load("GLESv2")
        egl, gl = self.egl, self.gl

        egl.eglGetDisplay.restype = ctypes.c_void_p
        egl.eglGetDisplay.argtypes = [ctypes.c_void_p]
        egl.eglInitialize.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_int),
                                       ctypes.POINTER(ctypes.c_int)]
        egl.eglChooseConfig.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_int),
                                         ctypes.POINTER(ctypes.c_void_p), ctypes.c_int,
                                         ctypes.POINTER(ctypes.c_int)]
        egl.eglCreateContext.restype = ctypes.c_void_p
        egl.eglCreateContext.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_void_p,
                                          ctypes.POINTER(ctypes.c_int)]
        egl.eglCreatePbufferSurface.restype = ctypes.c_void_p
        egl.eglCreatePbufferSurface.argtypes = [ctypes.c_void_p, ctypes.c_void_p,
                                                 ctypes.POINTER(ctypes.c_int)]
        egl.eglMakeCurrent.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_void_p,
                                        ctypes.c_void_p]
        egl.eglQueryString.restype = ctypes.c_char_p
        egl.eglQueryString.argtypes = [ctypes.c_void_p, ctypes.c_int]
        gl.glGetString.restype = ctypes.c_char_p
        gl.glGetShaderiv.argtypes = [ctypes.c_uint, ctypes.c_uint, ctypes.POINTER(ctypes.c_int)]
        gl.glGetProgramiv.argtypes = [ctypes.c_uint, ctypes.c_uint, ctypes.POINTER(ctypes.c_int)]
        gl.glGetUniformLocation.restype = ctypes.c_int
        gl.glGetUniformLocation.argtypes = [ctypes.c_uint, ctypes.c_char_p]
        gl.glUniform1f.argtypes = [ctypes.c_int, ctypes.c_float]
        gl.glUniform2f.argtypes = [ctypes.c_int, ctypes.c_float, ctypes.c_float]
        gl.glUniform3f.argtypes = [ctypes.c_int, ctypes.c_float, ctypes.c_float, ctypes.c_float]
        gl.glUniform1i.argtypes = [ctypes.c_int, ctypes.c_int]
        gl.glVertexAttribPointer.argtypes = [ctypes.c_uint, ctypes.c_int, ctypes.c_uint,
                                              ctypes.c_ubyte, ctypes.c_int, ctypes.c_void_p]

        dpy = egl.eglGetDisplay(EGL_DEFAULT_DISPLAY)
        if not dpy:
            raise RuntimeError("eglGetDisplay returned NULL")
        major, minor = ctypes.c_int(), ctypes.c_int()
        if not egl.eglInitialize(dpy, ctypes.byref(major), ctypes.byref(minor)):
            raise RuntimeError("eglInitialize failed")
        egl.eglBindAPI(EGL_OPENGL_ES_API)

        cfg_attribs = (ctypes.c_int * 13)(
            EGL_SURFACE_TYPE, EGL_PBUFFER_BIT, EGL_RENDERABLE_TYPE, EGL_OPENGL_ES2_BIT,
            EGL_RED_SIZE, 8, EGL_GREEN_SIZE, 8, EGL_BLUE_SIZE, 8, EGL_ALPHA_SIZE, 8, EGL_NONE)
        configs = (ctypes.c_void_p * 1)()
        n = ctypes.c_int()
        if not egl.eglChooseConfig(dpy, cfg_attribs, configs, 1, ctypes.byref(n)) or n.value < 1:
            raise RuntimeError("eglChooseConfig found no usable config")
        cfg = configs[0]

        ctx_attribs = (ctypes.c_int * 3)(EGL_CONTEXT_CLIENT_VERSION, 2, EGL_NONE)
        ctx = egl.eglCreateContext(dpy, cfg, EGL_NO_CONTEXT, ctx_attribs)
        if not ctx:
            raise RuntimeError("eglCreateContext failed")
        surf_attribs = (ctypes.c_int * 5)(EGL_WIDTH, 4, EGL_HEIGHT, 4, EGL_NONE)
        surf = egl.eglCreatePbufferSurface(dpy, cfg, surf_attribs)
        if not surf:
            raise RuntimeError("eglCreatePbufferSurface failed")
        if not egl.eglMakeCurrent(dpy, surf, surf, ctx):
            raise RuntimeError("eglMakeCurrent failed")

        self.dpy, self.surf, self.ctx = dpy, surf, ctx
        self.renderer = (gl.glGetString(0x1F01) or b"").decode()
        gl.glPixelStorei(GL_UNPACK_ALIGNMENT, 1)
        # Tracked so close() can explicitly glDelete* everything -- see close()'s
        # docstring for why this matters (eglDestroyContext alone isn't enough on lima).
        self._all_textures = []
        self._all_fbos = []
        self._all_programs = []
        self._all_shaders = []

    def close(self):
        """Tear down every GL object THEN the EGL context, so repeatedly starting/
        stopping GpuVision (e.g. toggling manual mode) doesn't leak a little more each
        cycle. `eglDestroyContext` alone is supposed to implicitly free everything
        associated with the context per the EGL/GL spec, but confirmed on hardware this
        isn't fully honored by `lima` (a reverse-engineered driver) -- toggling manual
        mode repeatedly grew RSS by ~3-4MB per cycle with only eglDestroyContext, not
        plateauing. Explicit glDelete* calls (more likely to be correctly implemented
        than the implicit context-teardown path) close that gap. Deletion must happen
        BEFORE releasing the context (eglMakeCurrent(NONE)) -- GL calls need a current
        context. Safe to call even if construction partially failed."""
        gl = self.gl
        try:
            if self._all_textures:
                arr = (ctypes.c_uint * len(self._all_textures))(*[t.value for t in self._all_textures])
                gl.glDeleteTextures(len(arr), arr)
            if self._all_fbos:
                arr = (ctypes.c_uint * len(self._all_fbos))(*[f.value for f in self._all_fbos])
                gl.glDeleteFramebuffers(len(arr), arr)
            for prog in self._all_programs:
                gl.glDeleteProgram(prog)
            for sh in self._all_shaders:
                gl.glDeleteShader(sh)
        except Exception:
            pass          # best-effort -- still try the EGL teardown below regardless
        egl = self.egl
        try:
            if getattr(self, "dpy", None):
                egl.eglMakeCurrent(self.dpy, 0, 0, 0)      # release the current context
                if getattr(self, "ctx", None):
                    egl.eglDestroyContext(self.dpy, self.ctx)
                if getattr(self, "surf", None):
                    egl.eglDestroySurface(self.dpy, self.surf)
                egl.eglTerminate(self.dpy)
        except Exception:
            pass          # best-effort -- the process/thread is going away regardless

    def compile_shader(self, kind, src):
        gl = self.gl
        sh = gl.glCreateShader(kind)
        buf = ctypes.c_char_p(src)
        length = ctypes.c_int(len(src))
        gl.glShaderSource(sh, 1, ctypes.byref(buf), ctypes.byref(length))
        gl.glCompileShader(sh)
        status = ctypes.c_int()
        gl.glGetShaderiv(sh, GL_COMPILE_STATUS, ctypes.byref(status))
        if not status.value:
            log = ctypes.create_string_buffer(1024)
            gl.glGetShaderInfoLog(sh, 1024, None, log)
            raise RuntimeError(f"shader compile error: {log.value.decode()}\n---\n{src.decode()}")
        self._all_shaders.append(sh)
        return sh

    def program(self, vs_src, fs_src, attribs=("pos",)):
        gl = self.gl
        vs = self.compile_shader(GL_VERTEX_SHADER, vs_src)
        fs = self.compile_shader(GL_FRAGMENT_SHADER, fs_src)
        prog = gl.glCreateProgram()
        gl.glAttachShader(prog, vs)
        gl.glAttachShader(prog, fs)
        for i, name in enumerate(attribs):
            gl.glBindAttribLocation(prog, i, name.encode())
        gl.glLinkProgram(prog)
        status = ctypes.c_int()
        gl.glGetProgramiv(prog, GL_LINK_STATUS, ctypes.byref(status))
        if not status.value:
            log = ctypes.create_string_buffer(1024)
            gl.glGetProgramInfoLog(prog, 1024, None, log)
            raise RuntimeError(f"program link error: {log.value.decode()}")
        self._all_programs.append(prog)
        return prog

    def make_texture(self, w, h, fmt=GL_RGBA, filt=GL_NEAREST):
        gl = self.gl
        tex = ctypes.c_uint()
        gl.glGenTextures(1, ctypes.byref(tex))
        gl.glBindTexture(GL_TEXTURE_2D, tex)
        gl.glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, filt)
        gl.glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, filt)
        gl.glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE)
        gl.glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE)
        gl.glTexImage2D(GL_TEXTURE_2D, 0, fmt, w, h, 0, fmt, GL_UNSIGNED_BYTE, None)
        self._all_textures.append(tex)
        return tex

    def make_fbo(self, w, h, fmt=GL_RGBA, filt=GL_NEAREST):
        gl = self.gl
        tex = self.make_texture(w, h, fmt, filt)
        fbo = ctypes.c_uint()
        gl.glGenFramebuffers(1, ctypes.byref(fbo))
        gl.glBindFramebuffer(GL_FRAMEBUFFER, fbo)
        gl.glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, tex, 0)
        status = gl.glCheckFramebufferStatus(GL_FRAMEBUFFER)
        if status != GL_FRAMEBUFFER_COMPLETE:
            raise RuntimeError(f"FBO incomplete: {hex(status)}")
        self._all_fbos.append(fbo)
        return tex, fbo, w, h


_QUAD_VS = b"""
attribute vec2 pos;
varying vec2 v_uv;
void main() {
    v_uv = pos * 0.5 + 0.5;
    gl_Position = vec4(pos, 0.0, 1.0);
}
"""

# For the browser-facing JPEG tee only: bakes in the empirically-determined column-mirror
# correction on the GPU (see gpu-vision memory) so the CPU never has to reorder 300K
# pixels/frame in Python. Internal passes (diff/threshold) use _QUAD_VS unflipped.
_FLIP_VS = b"""
attribute vec2 pos;
varying vec2 v_uv;
void main() {
    vec2 uv = pos * 0.5 + 0.5;
    v_uv = vec2(1.0 - uv.x, uv.y);
    gl_Position = vec4(pos, 0.0, 1.0);
}
"""

_YUYV_TO_RGB_FS = b"""
precision highp float;
varying vec2 v_uv;
uniform sampler2D tex;
uniform float half_width;
// The raw YUYV byte stream (Y0 U Y1 V per pixel pair) is uploaded AS an RGBA8 texture
// at half the source width -- one texel = one 4-byte YUYV pair = (R=Y0,G=U,B=Y1,A=V)
// exactly, no CPU repacking needed. This sidesteps GL_LUMINANCE_ALPHA, which `lima`
// (a reverse-engineered driver) mishandled in testing -- RGBA8 is the one format every
// GLES2 implementation is guaranteed to get right.
void main() {
    float col = floor(v_uv.x * half_width * 2.0);
    float pair = floor(col / 2.0);
    vec2 uv = vec2((pair + 0.5) / half_width, v_uv.y);
    vec4 t = texture2D(tex, uv);
    float is_odd = mod(col, 2.0);
    float Y = mix(t.r, t.b, is_odd) * 255.0;
    float U = t.g * 255.0 - 128.0;
    float V = t.a * 255.0 - 128.0;
    float r = (Y + 1.402 * V) / 255.0;
    float g = (Y - 0.344136 * U - 0.714136 * V) / 255.0;
    float b = (Y + 1.772 * U) / 255.0;
    gl_FragColor = vec4(clamp(r, 0.0, 1.0), clamp(g, 0.0, 1.0), clamp(b, 0.0, 1.0), 1.0);
}
"""


_DIFF_FS = b"""
precision mediump float;
varying vec2 v_uv;
uniform sampler2D cur_tex;
uniform sampler2D prev_tex;
// Same weighted-centroid packing as _THRESHOLD_FS below (R=magnitude, G=mag*x, B=mag*y)
// -- one reduction pass gives both the plain "how much changed" PIR scalar (R, averaged)
// AND the motion-saliency bounding center (G/R, B/R), for free.
void main() {
    vec3 d = abs(texture2D(cur_tex, v_uv).rgb - texture2D(prev_tex, v_uv).rgb);
    float m = max(d.r, max(d.g, d.b));
    gl_FragColor = vec4(m, m * v_uv.x, m * v_uv.y, 1.0);
}
"""

_THRESHOLD_FS = b"""
precision mediump float;
varying vec2 v_uv;
uniform sampler2D tex;
uniform vec3 target_color;      // 0..1 RGB
uniform float threshold;        // 0..1 max distance to count as a match
void main() {
    vec3 c = texture2D(tex, v_uv).rgb;
    float dist = length(c - target_color);
    float hit = step(dist, threshold);         // 1.0 if within threshold, else 0.0
    gl_FragColor = vec4(hit, hit * v_uv.x, hit * v_uv.y, 1.0);
}
"""

# Human-viewable version of the threshold mask -- reads just the R channel (the 0/1 hit
# value; G/B encode the centroid math for _optical_bumper-style consumers, not for
# display) and replicates it to grayscale: white = matches the tracked colour, black =
# doesn't. Combined with _FLIP_VS (not _QUAD_VS) so it comes out already mirror-
# corrected for the browser, same as the normal live-view tee.
_MASK_VIEW_FS = b"""
precision mediump float;
varying vec2 v_uv;
uniform sampler2D tex;
void main() {
    float hit = texture2D(tex, v_uv).r;
    gl_FragColor = vec4(hit, hit, hit, 1.0);
}
"""

_LUMA_FS = b"""
precision mediump float;
varying vec2 v_uv;
uniform sampler2D tex;
void main() {
    float l = dot(texture2D(tex, v_uv).rgb, vec3(0.299, 0.587, 0.114));
    gl_FragColor = vec4(l, l, l, 1.0);
}
"""

# Trivial passthrough -- used for the halving/reduction passes. GL_LINEAR minification
# on the source texture does the actual box-filter averaging: sampling a texel-center
# of a texture at HALF the source resolution lands exactly between 4 source texels, so
# bilinear filtering returns their average for free -- no manual multi-sampling needed.
_COPY_FS = b"""
precision mediump float;
varying vec2 v_uv;
uniform sampler2D tex;
void main() {
    gl_FragColor = texture2D(tex, v_uv);
}
"""


def plan_downsample_stages(w, h, min_size=24):
    """Just the (w,h) sequence a downsample chain would halve through -- no GL calls,
    so the caller can pre-allocate real FBOs for each stage ONCE outside the per-frame
    loop (see `build_downsample_chain`). Reusing the same tex/FBO IDs every frame is
    required -- allocating new ones per frame leaks GPU memory (found + fixed on
    hardware: an unbounded-growth OOM-kill after ~4 minutes, see gpu-vision memory)."""
    sizes = []
    cw, ch = w, h
    while cw > min_size or ch > min_size:
        cw, ch = max(1, cw // 2), max(1, ch // 2)
        sizes.append((cw, ch))
    return sizes


def build_downsample_chain(gl, w, h, min_size=24):
    """Allocate the persistent FBO chain ONCE (call during setup, not per-frame)."""
    return [gl.make_fbo(cw, ch, GL_RGBA, filt=GL_LINEAR)
            for cw, ch in plan_downsample_stages(w, h, min_size)]


def run_downsample_chain(gl, copy_prog, quad, src_tex, chain):
    """Render src_tex through the pre-allocated `chain` (from build_downsample_chain).
    Returns (tex, w, h) of the final small texture -- caller reads it back and finishes
    any remaining averaging on the CPU (trivially cheap at this size). Zero allocation."""
    g = gl.gl
    tex = src_tex
    dst_tex = dst_fbo = w = h = None
    for dst_tex, dst_fbo, w, h in chain:
        g.glActiveTexture(GL_TEXTURE0)
        g.glBindTexture(GL_TEXTURE_2D, tex)
        draw_fullscreen(gl, copy_prog, quad, dst_fbo, w, h)
        tex = dst_tex
    return tex, w, h


def readback(gl, w, h):
    g = gl.gl
    buf = (ctypes.c_ubyte * (w * h * 4))()
    g.glReadPixels(0, 0, w, h, GL_RGBA, GL_UNSIGNED_BYTE, buf)
    return buf


def readback_into(gl, w, h, buf):
    """Same as readback(), but writes into a caller-owned buffer instead of allocating
    a new one -- glReadPixels just overwrites it, no stale data risk. Callers with a
    fixed per-session size (every per-frame readback in this module) should
    pre-allocate once via `make_readback_buffer` and reuse this every tick, since the
    naive per-frame `readback()` allocation is real, avoidable churn (up to 1.2MB/frame
    for the full-resolution JPEG-tee readback while a viewer is watching)."""
    gl.gl.glReadPixels(0, 0, w, h, GL_RGBA, GL_UNSIGNED_BYTE, buf)
    return buf


def make_readback_buffer(w, h):
    return (ctypes.c_ubyte * (w * h * 4))()


def largest_blob_sums(pixels, w, h):
    """Connected-component labeling over the small downsampled hit-mask buffer
    (`pixels`, RGBA, R=hit-density G/B=hit*u/hit*v as packed by `_THRESHOLD_FS`) to find
    the LARGEST contiguous match region, instead of a single global weighted centroid
    that would blend together multiple separate matching blobs (e.g. the tracked ball
    plus an unrelated similarly-coloured patch elsewhere in frame) into one wrong
    average position. Returns that component's (sum_r, sum_g, sum_b), or (0,0,0) if no
    pixel matched at all. 8-connected (diagonal touches count as one blob).

    Cheap by construction: this buffer is only ~20x15 cells (the final stage of
    `plan_downsample_stages`'s halving chain), so a plain BFS flood-fill is trivial cost
    next to the GL passes already running every frame -- no need for anything fancier."""
    n = w * h
    visited = bytearray(n)
    best_r = best_g = best_b = 0
    for start in range(n):
        if visited[start] or pixels[start * 4] == 0:
            continue
        stack = [start]
        visited[start] = 1
        comp_r = comp_g = comp_b = 0
        while stack:
            idx = stack.pop()
            comp_r += pixels[idx * 4]
            comp_g += pixels[idx * 4 + 1]
            comp_b += pixels[idx * 4 + 2]
            x, y = idx % w, idx // w
            for dy in (-1, 0, 1):
                ny = y + dy
                if ny < 0 or ny >= h:
                    continue
                for dx in (-1, 0, 1):
                    if dx == 0 and dy == 0:
                        continue
                    nx = x + dx
                    if nx < 0 or nx >= w:
                        continue
                    nidx = ny * w + nx
                    if not visited[nidx] and pixels[nidx * 4] != 0:
                        visited[nidx] = 1
                        stack.append(nidx)
        if comp_r > best_r:
            best_r, best_g, best_b = comp_r, comp_g, comp_b
    return best_r, best_g, best_b


TJPF_RGBA = 7
TJSAMP_420 = 2
TJFLAG_FASTDCT = 0x0800


class JpegEncoder:
    """Minimal ctypes binding to libturbojpeg's compress-only API (tjCompress2) — same
    "raw ctypes over one focused native library" pattern as the rest of this module.
    Not thread-safe across handles; each GpuVision instance owns exactly one."""

    def __init__(self, quality=80):
        path = ctypes.util.find_library("turbojpeg") or "libturbojpeg.so.0"
        self.lib = ctypes.CDLL(path)
        self.lib.tjInitCompress.restype = ctypes.c_void_p
        # srcBuf is c_void_p (not c_char_p): lets us pass the pre-allocated readback
        # ctypes array directly (arrays auto-decay to a pointer for c_void_p args) --
        # c_char_p would force a bytes()+create_string_buffer double-copy of a
        # 640x480x4=1.2MB frame on every single encode, which is real, avoidable cost
        # while a browser viewer is watching (~15fps -> tens of MB/s of memcpy).
        self.lib.tjCompress2.argtypes = [
            ctypes.c_void_p, ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_int,
            ctypes.c_int, ctypes.POINTER(ctypes.POINTER(ctypes.c_ubyte)),
            ctypes.POINTER(ctypes.c_ulong), ctypes.c_int, ctypes.c_int, ctypes.c_int]
        self.lib.tjFree.argtypes = [ctypes.c_void_p]
        self.lib.tjDestroy.argtypes = [ctypes.c_void_p]
        self.handle = self.lib.tjInitCompress()
        if not self.handle:
            raise RuntimeError("tjInitCompress failed")
        self.quality = quality

    def close(self):
        """Free the compressor instance. A fresh JpegEncoder is created every time
        GpuVision._loop() (re)starts (e.g. toggling manual mode) -- without this, each
        cycle leaked a `tjInitCompress()` handle's internal buffers/tables (DCT/
        quantization tables, row buffers) with no way to reclaim them, forever.
        Confirmed a real contributor on hardware: RSS kept growing ~3-4MB/cycle even
        after GLContext.close() explicitly deleted every GL object, which pointed at a
        leak source outside the GL layer entirely."""
        try:
            if self.handle:
                self.lib.tjDestroy(self.handle)
                self.handle = None
        except Exception:
            pass

    def encode(self, rgba_buf, w, h):
        """rgba_buf: a ctypes array (e.g. from readback_into), NOT a bytes/bytearray --
        passed straight through to libjpeg-turbo with zero extra copies."""
        buf_ptr = ctypes.POINTER(ctypes.c_ubyte)()
        buf_size = ctypes.c_ulong(0)
        rc = self.lib.tjCompress2(
            self.handle, rgba_buf, w, w * 4, h, TJPF_RGBA,
            ctypes.byref(buf_ptr), ctypes.byref(buf_size),
            TJSAMP_420, self.quality, TJFLAG_FASTDCT)
        if rc != 0 or not buf_ptr:
            raise RuntimeError("tjCompress2 failed")
        try:
            return ctypes.string_at(buf_ptr, buf_size.value)
        finally:
            self.lib.tjFree(buf_ptr)


class GLContext(_GL):
    """Public alias — this is the reusable EGL/GLES2 context + shader helpers."""


def make_quad_vbo(gl):
    quad = (ctypes.c_float * 8)(-1, -1, 1, -1, -1, 1, 1, 1)
    return quad


def draw_fullscreen(gl, prog, quad, fbo, w, h):
    g = gl.gl
    g.glBindFramebuffer(GL_FRAMEBUFFER, fbo)
    g.glViewport(0, 0, w, h)
    g.glUseProgram(prog)
    g.glVertexAttribPointer(0, 2, GL_FLOAT, 0, 0, quad)
    g.glEnableVertexAttribArray(0)
    g.glDrawArrays(GL_TRIANGLE_STRIP, 0, 4)


class GpuVision:
    """Owns the camera (continuous YUYV capture) + the GLES2 pipeline. Runs entirely on
    its own background thread — never touch this from the ROS executor thread. Call
    `start()`/`stop()`. Thread-safe readouts via the `motion`/`target` properties."""

    def __init__(self, dev=None, width=640, height=480, fps=15, logger=None):
        self._dev = dev
        self._cfg = dict(width=width, height=height, fps=fps)
        self._log = logger or (lambda *_: None)
        self._thread = None
        self._run = False
        self._lock = threading.Lock()
        self._motion = 0.0
        self._motion_at = 0.0
        self._motion_center = None         # (x, y, magnitude) -- motion-saliency bounding center
        self._target = None
        self._target_at = 0.0
        self._target_color = None          # (r,g,b) 0..1, or None = tracking disabled
        self._target_thresh = 0.25
        # Blob-size gating: `target` is only reported when the matched fraction of the
        # frame (confidence) falls in [min, max] -- 0.0/1.0 by default (no filtering).
        # min_confidence rejects noise (a stray pixel or two matching by chance);
        # max_confidence rejects "everything matches" false locks, e.g. calibrating on a
        # colour that also happens to match a big wall/background area (a real failure
        # mode hit earlier this session). Both reset to the no-filtering defaults on a
        # fresh set_target_color() so a stale limit can't silently hide a new colour.
        self._blob_min_confidence = 0.0
        self._blob_max_confidence = 1.0
        self._intercept_rate = 0.0         # kinetic intercept: target confidence growth/sec
        self._luma = 0.0                   # flashlight/dark reflex: 0..1 average frame luminance
        self._luma_at = 0.0
        self._viewers = 0                  # browser viewers of the JPEG tee (ref-counted)
        self._jpeg = None
        self._jpeg_seq = 0
        self._jpeg_cond = threading.Condition()
        # Same shape as the above, for the "show me the tracking mask" debug view
        # (white = matches the calibrated target colour, black = doesn't) -- only
        # computed while a viewer is connected AND a target colour is set.
        self._mask_viewers = 0
        self._mask_jpeg = None
        self._mask_jpeg_seq = 0
        self._mask_jpeg_cond = threading.Condition()

    # ---- thread-safe readouts ----
    @property
    def motion_score(self):
        with self._lock:
            return self._motion

    @property
    def motion_center(self):
        """(x, y, magnitude) of where in frame motion is concentrated (0..1,
        top-left origin), or None if no recent motion. "Motion-saliency bounding
        center" -- orient toward movement before spending an LLM call on it."""
        with self._lock:
            return self._motion_center

    @property
    def target(self):
        """(x, y, confidence) in 0..1 image-normalized coords (top-left origin), or
        None if no target color is set or nothing currently matches it."""
        with self._lock:
            return self._target

    @property
    def intercept_rate(self):
        """Kinetic intercept alert: rate of growth (per second) of the tracked target's
        confidence/mask-area over the last few frames. High + sustained = the target is
        growing in frame, i.e. approaching the lens, not just present. 0 if no target
        is set or it isn't currently visible."""
        with self._lock:
            return self._intercept_rate

    @property
    def luma(self):
        """0..1 average frame luminance (flashlight/dark reflex)."""
        with self._lock:
            return self._luma

    @property
    def has_target_color(self):
        """Whether a target colour is currently calibrated -- distinct from `target`
        being None, which also happens when a colour IS set but nothing in frame
        matches it right now. The tracking-mask stream needs this distinction to give
        a clear "no target set" error instead of hanging forever waiting for a mask
        frame that will never be computed."""
        with self._lock:
            return self._target_color is not None

    def set_target_color(self, rgb, threshold=0.25):
        """rgb: (r,g,b) each 0..1, or None to disable tracking. Resets the blob-size
        limits to "no filtering" -- a fresh colour pick shouldn't be silently hidden by
        a min/max leftover from tuning a previous target."""
        with self._lock:
            self._target_color = tuple(rgb) if rgb is not None else None
            self._target_thresh = float(threshold)
            self._blob_min_confidence = 0.0
            self._blob_max_confidence = 1.0

    @property
    def blob_tuning(self):
        """(threshold, min_confidence, max_confidence) -- for the UI to sync its
        sliders to the current live values (e.g. after a fresh set_target_color reset)."""
        with self._lock:
            return (self._target_thresh, self._blob_min_confidence, self._blob_max_confidence)

    def set_blob_tuning(self, threshold=None, min_confidence=None, max_confidence=None):
        """Adjust matching sensitivity/size gating WITHOUT re-picking the target colour
        -- each arg left None keeps its current value. threshold = colour-distance
        tolerance (smaller = stricter colour match); min/max_confidence = the matched-
        fraction-of-frame range that counts as a valid lock (see the fields' comments
        in __init__ for why both directions matter)."""
        with self._lock:
            if threshold is not None:
                self._target_thresh = max(0.02, min(1.0, float(threshold)))
            if min_confidence is not None:
                self._blob_min_confidence = max(0.0, min(1.0, float(min_confidence)))
            if max_confidence is not None:
                self._blob_max_confidence = max(0.0, min(1.0, float(max_confidence)))
            if self._blob_max_confidence < self._blob_min_confidence:
                self._blob_max_confidence = self._blob_min_confidence

    # ---- browser live-view tee (ref-counted, mirrors mjpeg_camera.CameraStream) ----
    def add_viewer(self):
        with self._jpeg_cond:
            self._viewers += 1

    def remove_viewer(self):
        with self._jpeg_cond:
            self._viewers = max(0, self._viewers - 1)

    def get_frame(self, last_seq, timeout=5.0):
        """Block until a JPEG frame newer than last_seq; return (seq, jpeg|None). Same
        name/shape as mjpeg_camera.CameraStream.get_frame -- lets web_server.py use
        whichever one is active as a drop-in `self._cam` with no branching elsewhere."""
        with self._jpeg_cond:
            if not self._jpeg_cond.wait_for(
                    lambda: self._jpeg_seq != last_seq or not self._run, timeout):
                return last_seq, None
            return self._jpeg_seq, self._jpeg

    def running(self):
        return self._run

    # ---- tracking-mask debug view (same ref-counted shape as the normal tee) ----
    def add_mask_viewer(self):
        with self._mask_jpeg_cond:
            self._mask_viewers += 1

    def remove_mask_viewer(self):
        with self._mask_jpeg_cond:
            self._mask_viewers = max(0, self._mask_viewers - 1)

    def get_mask_frame(self, last_seq, timeout=5.0):
        """Same shape as get_frame(), for the tracking-mask stream. Returns (seq, None)
        if no target colour is set yet (nothing has ever been computed) -- callers
        should treat that the same as "camera failed" and stop retrying/show an error,
        not spin forever waiting for a frame that will never come."""
        with self._mask_jpeg_cond:
            if not self._mask_jpeg_cond.wait_for(
                    lambda: self._mask_jpeg_seq != last_seq or not self._run, timeout):
                return last_seq, None
            return self._mask_jpeg_seq, self._mask_jpeg

    def start(self):
        if self._thread and self._thread.is_alive():
            return
        self._run = True
        self._thread = threading.Thread(target=self._loop, daemon=True, name="gpu_vision")
        self._thread.start()

    def stop(self):
        self._run = False
        if self._thread:
            self._thread.join(timeout=2.0)

    def _loop(self):
        try:
            gl = GLContext()
            self._log(f"gpu_vision: GL context up, renderer={gl.renderer}")
            if "mali" not in gl.renderer.lower():
                # Mesa's EGL_PLATFORM=surfaceless silently hands back a SOFTWARE
                # renderer (llvmpipe) when there's no DRM render node instead of
                # erroring -- confirmed on hardware 2026-07-11 (see memory
                # lima-boot-load-bug) after `lima` failed to auto-load at boot. That
                # means "GPU" vision silently runs on the CPU with zero other symptom
                # besides this string -- make it impossible to miss in the logs.
                self._log(f"gpu_vision: *** WARNING *** renderer '{gl.renderer}' is NOT "
                          f"the Mali-450 hardware driver -- running on a SOFTWARE "
                          f"rasterizer instead. Check `lsmod | grep lima` and "
                          f"`ls /dev/dri` on the board; `sudo modprobe lima` if missing.")
        except Exception as exc:
            self._log(f"gpu_vision: EGL/GLES init failed: {exc}")
            self._run = False    # so running() doesn't lie -- the thread is exiting
            return

        dev = self._dev or mjpeg_camera.find_camera()
        if not dev:
            self._log("gpu_vision: no camera found")
            gl.close()
            self._run = False
            return
        # Retry a handful of times: when switching OFF manual mode, the direct
        # CameraStream backend releases the V4L2 device ASYNCHRONOUSLY in its own
        # thread (a viewer's remove_viewer() just sets a flag; the actual close()
        # happens once that thread notices) -- confirmed on hardware this creates a
        # real, reproducible "[Errno 16] Device or resource busy" race right after
        # toggling manual mode off. The busy condition is transient (clears within a
        # few hundred ms once the other backend's thread catches up), so retry instead
        # of giving up on the very first attempt.
        cam = None
        last_exc = None
        for attempt in range(6):
            try:
                cam = mjpeg_camera.MjpegCamera(dev, fourcc=mjpeg_camera.FOURCC_YUYV, **self._cfg)
                break
            except Exception as exc:
                last_exc = exc
                if not self._run:
                    break     # stop() was called while we were retrying -- give up cleanly
                time.sleep(0.3)
        if cam is None:
            self._log(f"gpu_vision: camera open (YUYV) failed after retries: {last_exc}")
            gl.close()
            self._run = False
            return
        self._log(f"gpu_vision: capturing {dev} YUYV {cam.width}x{cam.height} "
                   f"bytesperline={cam.bytesperline}")
        W, H = cam.width, cam.height

        yuyv_prog = gl.program(_QUAD_VS, _YUYV_TO_RGB_FS)
        u_half_width = gl.gl.glGetUniformLocation(yuyv_prog, b"half_width")
        copy_prog = gl.program(_QUAD_VS, _COPY_FS)
        diff_prog = gl.program(_QUAD_VS, _DIFF_FS)
        u_cur = gl.gl.glGetUniformLocation(diff_prog, b"cur_tex")
        u_prev = gl.gl.glGetUniformLocation(diff_prog, b"prev_tex")
        thresh_prog = gl.program(_QUAD_VS, _THRESHOLD_FS)
        u_tex = gl.gl.glGetUniformLocation(thresh_prog, b"tex")
        u_target = gl.gl.glGetUniformLocation(thresh_prog, b"target_color")
        u_thresh = gl.gl.glGetUniformLocation(thresh_prog, b"threshold")
        luma_prog = gl.program(_QUAD_VS, _LUMA_FS)
        u_luma_tex = gl.gl.glGetUniformLocation(luma_prog, b"tex")
        flip_prog = gl.program(_FLIP_VS, _COPY_FS)
        mask_view_prog = gl.program(_FLIP_VS, _MASK_VIEW_FS)   # mirror-corrected, like flip_prog
        u_mask_tex = gl.gl.glGetUniformLocation(mask_view_prog, b"tex")
        quad = make_quad_vbo(gl)

        yuyv_tex = gl.make_texture(W // 2, H, GL_RGBA)
        # Ping-pong RGB buffers: each tick writes into the OTHER one, so "the other
        # buffer" is always last tick's frame -- no explicit copy needed for PIR diff.
        rgb = [gl.make_fbo(W, H, GL_RGBA) for _ in range(2)]
        diff_tex, diff_fbo, _, _ = gl.make_fbo(W, H, GL_RGBA, filt=GL_LINEAR)
        thresh_tex, thresh_fbo, _, _ = gl.make_fbo(W, H, GL_RGBA, filt=GL_LINEAR)
        luma_tex, luma_fbo, _, _ = gl.make_fbo(W, H, GL_RGBA, filt=GL_LINEAR)
        flip_tex, flip_fbo, _, _ = gl.make_fbo(W, H, GL_RGBA)
        mask_flip_tex, mask_flip_fbo, _, _ = gl.make_fbo(W, H, GL_RGBA)
        # Pre-allocated ONCE, reused every frame -- see build_downsample_chain's
        # docstring for why (a real leak, found + fixed on hardware).
        diff_chain = build_downsample_chain(gl, W, H)
        thresh_chain = build_downsample_chain(gl, W, H)
        luma_chain = build_downsample_chain(gl, W, H)
        # Readback buffers, also pre-allocated once (same reasoning as the FBO chains --
        # per-frame ctypes allocation churn is real, avoidable cost; a chain's final
        # stage size is fixed for the whole session since W/H never change).
        small_w, small_h = plan_downsample_stages(W, H)[-1]
        diff_buf = make_readback_buffer(small_w, small_h)
        thresh_buf = make_readback_buffer(small_w, small_h)
        luma_buf = make_readback_buffer(small_w, small_h)
        flip_buf = make_readback_buffer(W, H)
        mask_flip_buf = make_readback_buffer(W, H)
        cur_idx = 0
        have_prev = False
        target_hist = []           # kinetic intercept: [(t, confidence), ...] last few samples
        try:
            jpeg_enc = JpegEncoder()
        except Exception as exc:
            self._log(f"gpu_vision: JPEG encoder unavailable ({exc}); browser tee disabled")
            jpeg_enc = None

        g = gl.gl
        period = 1.0 / max(1, self._cfg.get("fps", 15))
        next_t = time.monotonic()
        while self._run:
            try:
                buf = cam.read(1000)
                if buf is None:
                    continue
                while True:
                    extra = cam.read(0)
                    if extra is None:
                        break
                    buf = extra
            except Exception as exc:
                self._log(f"gpu_vision: capture read error: {exc}")
                self._run = False    # so running() doesn't lie -- the loop is exiting
                break

            g.glActiveTexture(GL_TEXTURE0)
            g.glBindTexture(GL_TEXTURE_2D, yuyv_tex)
            # `buf` (from cam.read()) is already a plain Python bytes object; ctypes
            # converts it to a pointer automatically for this untyped call (no
            # glTexImage2D.argtypes declared) -- wrapping it in create_string_buffer
            # first would just be a second, unnecessary copy of the whole frame.
            g.glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, W // 2, H, 0,
                            GL_RGBA, GL_UNSIGNED_BYTE, buf)
            g.glUseProgram(yuyv_prog)          # uniform calls target the BOUND program
            g.glUniform1f(u_half_width, float(W // 2))
            cur_tex, cur_fbo, _, _ = rgb[cur_idx]
            draw_fullscreen(gl, yuyv_prog, quad, cur_fbo, W, H)

            if have_prev:
                prev_tex, _, _, _ = rgb[1 - cur_idx]
                g.glUseProgram(diff_prog)
                g.glActiveTexture(GL_TEXTURE0)
                g.glBindTexture(GL_TEXTURE_2D, cur_tex)
                g.glActiveTexture(GL_TEXTURE0 + 1)
                g.glBindTexture(GL_TEXTURE_2D, prev_tex)
                g.glUniform1i(u_cur, 0)
                g.glUniform1i(u_prev, 1)
                draw_fullscreen(gl, diff_prog, quad, diff_fbo, W, H)

                small_tex, sw, sh = run_downsample_chain(gl, copy_prog, quad, diff_tex, diff_chain)
                pixels = readback_into(gl, sw, sh, diff_buf)
                n = sw * sh
                sum_r = sum_g = sum_b = 0
                for i in range(n):
                    sum_r += pixels[i * 4]
                    sum_g += pixels[i * 4 + 1]
                    sum_b += pixels[i * 4 + 2]
                score = (sum_r / n) / 255.0
                # Motion-saliency bounding center: same weighted-centroid trick as blob
                # tracking, "for free" from the same reduction pass (see _DIFF_FS).
                if sum_r > 0:
                    center = (1.0 - (sum_g / sum_r), sum_b / sum_r, score)
                else:
                    center = None
                with self._lock:
                    self._motion = score
                    self._motion_at = time.monotonic()
                    self._motion_center = center

            with self._lock:
                target_color = self._target_color
                target_thresh = self._target_thresh
                blob_min = self._blob_min_confidence
                blob_max = self._blob_max_confidence
            if target_color is not None:
                g.glUseProgram(thresh_prog)
                g.glActiveTexture(GL_TEXTURE0)
                g.glBindTexture(GL_TEXTURE_2D, cur_tex)
                g.glUniform1i(u_tex, 0)
                g.glUniform3f(u_target, *target_color)
                g.glUniform1f(u_thresh, target_thresh)
                draw_fullscreen(gl, thresh_prog, quad, thresh_fbo, W, H)

                tsmall_tex, tsw, tsh = run_downsample_chain(gl, copy_prog, quad, thresh_tex, thresh_chain)
                tpixels = readback_into(gl, tsw, tsh, thresh_buf)
                n2 = tsw * tsh
                # Largest-blob selection (not a global sum over every matching pixel) --
                # see largest_blob_sums's docstring. confidence stays normalized by the
                # WHOLE frame's cell count (not just the blob's), so it keeps meaning
                # "fraction of the frame the tracked blob covers" -- unchanged scale for
                # the blob_min/max tuning sliders and the UI's locked/searching %.
                sum_r, sum_g, sum_b = largest_blob_sums(tpixels, tsw, tsh)
                confidence = sum_r / (n2 * 255.0)
                # Blob-size gating: a valid lock needs BOTH a nonzero mask (a centroid to
                # even compute) AND confidence inside [blob_min, blob_max] -- rejects
                # noise (too small, below blob_min) and "matched almost the whole frame"
                # false locks (too big, above blob_max, e.g. a colour that also matches
                # a wall/background). Note `confidence` itself stays un-gated below for
                # the kinetic-intercept trend, which should keep tracking growth even
                # while still under blob_min (approaching from far away).
                if sum_r > 0 and blob_min <= confidence <= blob_max:
                    raw_u = sum_g / sum_r
                    raw_v = sum_b / sum_r
                    # Empirically-determined mirror correction (see gpu-vision memory):
                    # columns come back reversed, rows do not.
                    target = (1.0 - raw_u, raw_v, confidence)
                else:
                    target = None
                with self._lock:
                    self._target = target
                    self._target_at = time.monotonic()

                # Kinetic intercept alert: track the blob's confidence (~mask area) over
                # the last few frames; a fast, sustained rise means it's growing in frame
                # -- i.e. approaching the lens, not just present. Pure Python over
                # already-computed numbers, no extra shader pass.
                now_t = time.monotonic()
                target_hist.append((now_t, confidence))
                del target_hist[:-5]
                expansion = 0.0
                if len(target_hist) >= 3:
                    t0, c0 = target_hist[0]
                    dt = now_t - t0
                    if dt > 0.05:
                        expansion = max(0.0, (confidence - c0) / dt)
                with self._lock:
                    self._intercept_rate = expansion
            else:
                target_hist.clear()
                with self._lock:
                    self._intercept_rate = 0.0

            # Flashlight/dark reflex: global average luminance, same reduction machinery.
            g.glUseProgram(luma_prog)
            g.glActiveTexture(GL_TEXTURE0)
            g.glBindTexture(GL_TEXTURE_2D, cur_tex)
            g.glUniform1i(u_luma_tex, 0)
            draw_fullscreen(gl, luma_prog, quad, luma_fbo, W, H)
            lsmall_tex, lsw, lsh = run_downsample_chain(gl, copy_prog, quad, luma_tex, luma_chain)
            lpixels = readback_into(gl, lsw, lsh, luma_buf)
            ln = lsw * lsh
            lsum = 0
            for i in range(ln):
                lsum += lpixels[i * 4]
            luma = (lsum / ln) / 255.0
            with self._lock:
                self._luma = luma
                self._luma_at = time.monotonic()

            with self._jpeg_cond:
                want_jpeg = self._viewers > 0
            if want_jpeg and jpeg_enc is not None:
                g.glActiveTexture(GL_TEXTURE0)
                g.glBindTexture(GL_TEXTURE_2D, cur_tex)
                draw_fullscreen(gl, flip_prog, quad, flip_fbo, W, H)
                pixels = readback_into(gl, W, H, flip_buf)
                try:
                    jpeg = jpeg_enc.encode(pixels, W, H)
                    with self._jpeg_cond:
                        self._jpeg = jpeg
                        self._jpeg_seq += 1
                        self._jpeg_cond.notify_all()
                except Exception as exc:
                    self._log(f"gpu_vision: JPEG encode failed: {exc}")

            # Tracking-mask debug view: same viewer-gated shape as the tee above, but
            # also needs a target colour actually set -- thresh_tex only has meaningful
            # content when the threshold pass above ran (target_color is not None).
            with self._mask_jpeg_cond:
                want_mask = self._mask_viewers > 0
            if want_mask and jpeg_enc is not None and target_color is not None:
                g.glUseProgram(mask_view_prog)
                g.glActiveTexture(GL_TEXTURE0)
                g.glBindTexture(GL_TEXTURE_2D, thresh_tex)
                g.glUniform1i(u_mask_tex, 0)
                draw_fullscreen(gl, mask_view_prog, quad, mask_flip_fbo, W, H)
                mask_pixels = readback_into(gl, W, H, mask_flip_buf)
                try:
                    mask_jpeg = jpeg_enc.encode(mask_pixels, W, H)
                    with self._mask_jpeg_cond:
                        self._mask_jpeg = mask_jpeg
                        self._mask_jpeg_seq += 1
                        self._mask_jpeg_cond.notify_all()
                except Exception as exc:
                    self._log(f"gpu_vision: mask JPEG encode failed: {exc}")

            g.glFinish()
            cur_idx = 1 - cur_idx
            have_prev = True

            next_t += period
            dt = next_t - time.monotonic()
            if dt > 0:
                time.sleep(dt)
            else:
                next_t = time.monotonic()

        cam.close()
        if jpeg_enc is not None:
            jpeg_enc.close()    # free the turbojpeg compressor -- see JpegEncoder.close()
        gl.close()              # release the EGL context's ~70MB, not just the camera fd
        with self._jpeg_cond:
            self._jpeg_cond.notify_all()       # wake any blocked get_jpeg() callers
        with self._mask_jpeg_cond:
            self._mask_jpeg_cond.notify_all()  # wake any blocked get_mask_frame() callers
        self._log("gpu_vision: stopped")


def _test_pir(seconds=8.0):
    """Manual on-hardware verification: run the full GpuVision loop and print the
    motion score every 0.5s. Static scene should stay near-zero throughout."""
    gv = GpuVision(logger=print)
    gv.start()
    t0 = time.monotonic()
    while time.monotonic() - t0 < seconds:
        time.sleep(0.5)
        print(f"t={time.monotonic()-t0:5.1f}s  motion_score={gv.motion_score:.4f}")
    gv.stop()


def _test_jpeg(out="/tmp/gpu_vision_tee.jpg", seconds=3.0):
    """Manual on-hardware verification: register as a viewer, wait for a JPEG frame,
    save it. Confirms the browser-facing tee (readback + flip + turbojpeg) works."""
    gv = GpuVision(logger=print)
    gv.start()
    gv.add_viewer()
    seq, jpeg = 0, None
    t0 = time.monotonic()
    while time.monotonic() - t0 < seconds and jpeg is None:
        seq, jpeg = gv.get_frame(0, timeout=1.0)
    gv.remove_viewer()
    gv.stop()
    if jpeg is None:
        print("FAIL: no JPEG frame produced")
        raise SystemExit(1)
    with open(out, "wb") as f:
        f.write(jpeg)
    print(f"wrote {out} ({len(jpeg)} bytes)")


def _test_blob(r, g, b, thresh=0.25, seconds=6.0):
    """Manual on-hardware verification: track a given target colour, print (x,y,conf)."""
    gv = GpuVision(logger=print)
    gv.set_target_color((r, g, b), thresh)
    gv.start()
    t0 = time.monotonic()
    while time.monotonic() - t0 < seconds:
        time.sleep(0.5)
        print(f"t={time.monotonic()-t0:5.1f}s  target={gv.target}")
    gv.stop()


if __name__ == "__main__":
    import sys

    if len(sys.argv) > 1 and sys.argv[1] == "pir":
        _test_pir(float(sys.argv[2]) if len(sys.argv) > 2 else 8.0)
        raise SystemExit(0)
    if len(sys.argv) > 1 and sys.argv[1] == "blob":
        r, g, b = (float(x) for x in sys.argv[2:5])
        _test_blob(r, g, b)
        raise SystemExit(0)
    if len(sys.argv) > 1 and sys.argv[1] == "jpeg":
        _test_jpeg()
        raise SystemExit(0)
    if len(sys.argv) > 1 and sys.argv[1] == "mask":
        # Dump the full-res threshold mask (not downsampled) as a grayscale PPM, so it
        # can be visually compared against the actual scene -- the most direct check.
        tgt_r, tgt_g, tgt_b = (float(x) for x in sys.argv[2:5])
        thresh = float(sys.argv[5]) if len(sys.argv) > 5 else 0.25
        out = sys.argv[6] if len(sys.argv) > 6 else "/tmp/gpu_vision_mask.ppm"
        gl = GLContext()
        dev = mjpeg_camera.find_camera()
        cam = mjpeg_camera.MjpegCamera(dev, fourcc=mjpeg_camera.FOURCC_YUYV, width=640, height=480, fps=15)
        for _ in range(5):
            buf = cam.read(1000)
        W, H = cam.width, cam.height
        yuyv_prog = gl.program(_QUAD_VS, _YUYV_TO_RGB_FS)
        u_half_width = gl.gl.glGetUniformLocation(yuyv_prog, b"half_width")
        thresh_prog = gl.program(_QUAD_VS, _THRESHOLD_FS)
        u_tex = gl.gl.glGetUniformLocation(thresh_prog, b"tex")
        u_target = gl.gl.glGetUniformLocation(thresh_prog, b"target_color")
        u_thresh = gl.gl.glGetUniformLocation(thresh_prog, b"threshold")
        quad = make_quad_vbo(gl)
        yuyv_tex = gl.make_texture(W // 2, H, GL_RGBA)
        rgb_tex, rgb_fbo, _, _ = gl.make_fbo(W, H, GL_RGBA)
        mask_tex, mask_fbo, _, _ = gl.make_fbo(W, H, GL_RGBA)
        g = gl.gl
        g.glBindTexture(GL_TEXTURE_2D, yuyv_tex)
        g.glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, W // 2, H, 0, GL_RGBA, GL_UNSIGNED_BYTE, buf)
        g.glUseProgram(yuyv_prog)
        g.glUniform1f(u_half_width, float(W // 2))
        draw_fullscreen(gl, yuyv_prog, quad, rgb_fbo, W, H)
        g.glUseProgram(thresh_prog)
        g.glActiveTexture(GL_TEXTURE0)
        g.glBindTexture(GL_TEXTURE_2D, rgb_tex)
        g.glUniform1i(u_tex, 0)
        g.glUniform3f(u_target, tgt_r, tgt_g, tgt_b)
        g.glUniform1f(u_thresh, thresh)
        draw_fullscreen(gl, thresh_prog, quad, mask_fbo, W, H)
        g.glFinish()
        pixels = readback(gl, W, H)
        cam.close()
        with open(out, "wb") as f:
            f.write(f"P6\n{W} {H}\n255\n".encode())
            row_bytes = W * 4
            for row in range(H):
                start = row * row_bytes
                for col in range(W - 1, -1, -1):
                    px = start + col * 4
                    v = pixels[px]              # R channel = hit mask
                    f.write(bytes([v, v, v]))
        print(f"wrote {out}")
        raise SystemExit(0)

    # Manual on-hardware verification: capture one converted frame, dump it as a PPM so
    # it can be scp'd back and visually inspected. Not part of the ROS wiring.
    gv_dev = sys.argv[1] if len(sys.argv) > 1 else None
    gl = GLContext()
    print(f"renderer={gl.renderer}")
    dev = gv_dev or mjpeg_camera.find_camera()
    print(f"device={dev}")
    cam = mjpeg_camera.MjpegCamera(dev, fourcc=mjpeg_camera.FOURCC_YUYV, width=640, height=480, fps=15)
    print(f"capturing {cam.width}x{cam.height} bytesperline={cam.bytesperline}")
    buf = None
    for _ in range(5):           # skip a few frames to let exposure settle
        buf = cam.read(1000)
    assert buf is not None, "no frame captured"
    print(f"got frame: {len(buf)} bytes (expect {cam.width*cam.height*2})")

    yuyv_prog = gl.program(_QUAD_VS, _YUYV_TO_RGB_FS)
    u_half_width = gl.gl.glGetUniformLocation(yuyv_prog, b"half_width")
    quad = make_quad_vbo(gl)
    W, H = cam.width, cam.height
    yuyv_tex = gl.make_texture(W // 2, H, GL_RGBA)
    rgb_tex, rgb_fbo, _, _ = gl.make_fbo(W, H, GL_RGBA)

    g = gl.gl
    g.glBindTexture(GL_TEXTURE_2D, yuyv_tex)
    g.glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, W // 2, H, 0,
                    GL_RGBA, GL_UNSIGNED_BYTE, buf)
    g.glUseProgram(yuyv_prog)          # uniform calls target the BOUND program
    g.glUniform1f(u_half_width, float(W // 2))
    draw_fullscreen(gl, yuyv_prog, quad, rgb_fbo, W, H)
    g.glFinish()

    pixels = (ctypes.c_ubyte * (W * H * 4))()
    g.glReadPixels(0, 0, W, H, GL_RGBA, GL_UNSIGNED_BYTE, pixels)
    cam.close()

    out = sys.argv[2] if len(sys.argv) > 2 else "/tmp/gpu_vision_test.ppm"
    with open(out, "wb") as f:
        f.write(f"P6\n{W} {H}\n255\n".encode())
        # Empirically determined against a known-good reference frame on real hardware:
        # rows come back already top-down, but mirrored left-right -- reverse columns,
        # not rows (see gpu-vision memory for how this was derived).
        row_bytes = W * 4
        for row in range(H):
            start = row * row_bytes
            for col in range(W - 1, -1, -1):
                px = start + col * 4
                f.write(bytes([pixels[px], pixels[px + 1], pixels[px + 2]]))
    print(f"wrote {out}")
