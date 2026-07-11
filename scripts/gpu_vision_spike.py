#!/usr/bin/env python3
"""Phase-0 hardware spike for the GPU-vision backlog plan (see the plan file / memory
`gpu-vision-features-todo`). Zero-dependency (raw ctypes against libEGL/libGLESv2, same
zero-dep philosophy as `web_control/mjpeg_camera.py`'s raw V4L2 ioctls) so it needs nothing
beyond the Mesa/EGL/GBM apt packages already installed by `deploy/sbc-setup.sh`.

Creates a headless EGL/GLES2 context (Mesa's "surfaceless" platform — no X/Wayland, no
on-screen surface needed, backed by /dev/dri/renderD128), prints which extensions the
`lima` driver actually supports, times one real FBO threshold-shader pass, and prints
VmRSS at each stage. Run on the robot: `python3 scripts/gpu_vision_spike.py`.

This is a one-off manual probe, not a pixi task and not wired into `pixi run smoke`.
"""
import ctypes
import ctypes.util
import os
import sys
import time

os.environ.setdefault("EGL_PLATFORM", "surfaceless")


def rss_mb():
    with open("/proc/self/status") as f:
        for line in f:
            if line.startswith("VmRSS:"):
                return int(line.split()[1]) / 1024.0
    return -1.0


def report(stage):
    print(f"  [{stage}] VmRSS = {rss_mb():.1f} MB")


# ---- EGL / GLES2 constants (khrplatform / EGL 1.4-1.5 / GLES2) ----
EGL_DEFAULT_DISPLAY = 0
EGL_NO_CONTEXT = 0
EGL_NO_SURFACE = 0
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
EGL_EXTENSIONS = 0x3055

GL_FRAGMENT_SHADER = 0x8B30
GL_VERTEX_SHADER = 0x8B31
GL_COMPILE_STATUS = 0x8B81
GL_LINK_STATUS = 0x8B82
GL_FRAMEBUFFER = 0x8D40
GL_COLOR_ATTACHMENT0 = 0x8CE0
GL_TEXTURE_2D = 0x0DE1
GL_RGBA = 0x1908
GL_UNSIGNED_BYTE = 0x1401
GL_FRAMEBUFFER_COMPLETE = 0x8CD5
GL_TRIANGLE_STRIP = 0x0005
GL_FLOAT = 0x1406
GL_EXTENSIONS = 0x1F03
GL_TEXTURE_MIN_FILTER = 0x2801
GL_TEXTURE_MAG_FILTER = 0x2800
GL_NEAREST = 0x2600
GL_TEXTURE_WRAP_S = 0x2802
GL_TEXTURE_WRAP_T = 0x2803
GL_CLAMP_TO_EDGE = 0x812F


def load(name):
    path = ctypes.util.find_library(name) or f"lib{name}.so.1"
    return ctypes.CDLL(path)


def main():
    print("=== GPU vision Phase-0 spike ===")
    report("baseline")

    try:
        egl = load("EGL")
        gles = load("GLESv2")
    except OSError as e:
        print(f"FAIL: could not load libEGL/libGLESv2: {e}")
        print("  -> Mesa/EGL/GBM packages missing? see deploy/sbc-setup.sh Phase 0 step")
        sys.exit(1)
    report("post-import")

    # Correct pointer-sized return types (ctypes defaults to c_int, which TRUNCATES
    # 64-bit pointers -- this is the single most common way this kind of script silently
    # crashes/segfaults on aarch64).
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
    gles.glGetString.restype = ctypes.c_char_p

    dpy = egl.eglGetDisplay(EGL_DEFAULT_DISPLAY)
    if not dpy:
        print("FAIL: eglGetDisplay returned NULL (no surfaceless/GBM platform found)")
        sys.exit(1)

    major, minor = ctypes.c_int(), ctypes.c_int()
    if not egl.eglInitialize(dpy, ctypes.byref(major), ctypes.byref(minor)):
        print("FAIL: eglInitialize failed")
        sys.exit(1)
    print(f"  EGL version: {major.value}.{minor.value}")

    egl_ext = (egl.eglQueryString(dpy, EGL_EXTENSIONS) or b"").decode()
    has_dmabuf = "EGL_EXT_image_dma_buf_import" in egl_ext
    print(f"  EGL_EXT_image_dma_buf_import: {has_dmabuf}")

    egl.eglBindAPI(EGL_OPENGL_ES_API)

    cfg_attribs = (ctypes.c_int * 13)(
        EGL_SURFACE_TYPE, EGL_PBUFFER_BIT,
        EGL_RENDERABLE_TYPE, EGL_OPENGL_ES2_BIT,
        EGL_RED_SIZE, 8, EGL_GREEN_SIZE, 8, EGL_BLUE_SIZE, 8, EGL_ALPHA_SIZE, 8,
        EGL_NONE)
    configs = (ctypes.c_void_p * 1)()
    n = ctypes.c_int()
    if not egl.eglChooseConfig(dpy, cfg_attribs, configs, 1, ctypes.byref(n)) or n.value < 1:
        print("FAIL: eglChooseConfig found no usable config")
        sys.exit(1)
    cfg = configs[0]

    ctx_attribs = (ctypes.c_int * 3)(EGL_CONTEXT_CLIENT_VERSION, 2, EGL_NONE)
    ctx = egl.eglCreateContext(dpy, cfg, EGL_NO_CONTEXT, ctx_attribs)
    if not ctx:
        print("FAIL: eglCreateContext failed")
        sys.exit(1)

    surf_attribs = (ctypes.c_int * 5)(EGL_WIDTH, 4, EGL_HEIGHT, 4, EGL_NONE)
    surf = egl.eglCreatePbufferSurface(dpy, cfg, surf_attribs)
    if not surf:
        print("FAIL: eglCreatePbufferSurface failed")
        sys.exit(1)

    if not egl.eglMakeCurrent(dpy, surf, surf, ctx):
        print("FAIL: eglMakeCurrent failed")
        sys.exit(1)
    report("post-context")

    gl_ext = (gles.glGetString(GL_EXTENSIONS) or b"").decode()
    has_ext_image = "GL_OES_EGL_image_external" in gl_ext
    renderer = (gles.glGetString(0x1F01) or b"").decode()   # GL_RENDERER
    version = (gles.glGetString(0x1F02) or b"").decode()    # GL_VERSION
    print(f"  GL_RENDERER: {renderer}")
    print(f"  GL_VERSION: {version}")
    print(f"  GL_OES_EGL_image_external: {has_ext_image}")

    # ---- one real FBO threshold pass: 640x480 texture in, FBO out, glReadPixels back ----
    vs_src = b"""
    attribute vec2 pos;
    void main() { gl_Position = vec4(pos, 0.0, 1.0); }
    """
    fs_src = b"""
    precision mediump float;
    void main() { gl_FragColor = vec4(1.0, 0.0, 0.0, 1.0); }
    """

    def compile_shader(kind, src):
        sh = gles.glCreateShader(kind)
        buf = ctypes.c_char_p(src)
        length = ctypes.c_int(len(src))
        gles.glShaderSource(sh, 1, ctypes.byref(buf), ctypes.byref(length))
        gles.glCompileShader(sh)
        status = ctypes.c_int()
        gles.glGetShaderiv(sh, GL_COMPILE_STATUS, ctypes.byref(status))
        if not status.value:
            log = ctypes.create_string_buffer(512)
            gles.glGetShaderInfoLog(sh, 512, None, log)
            print(f"FAIL: shader compile error: {log.value.decode()}")
            sys.exit(1)
        return sh

    gles.glGetShaderiv.argtypes = [ctypes.c_uint, ctypes.c_uint, ctypes.POINTER(ctypes.c_int)]
    vs = compile_shader(GL_VERTEX_SHADER, vs_src)
    fs = compile_shader(GL_FRAGMENT_SHADER, fs_src)
    prog = gles.glCreateProgram()
    gles.glAttachShader(prog, vs)
    gles.glAttachShader(prog, fs)
    gles.glBindAttribLocation(prog, 0, b"pos")
    gles.glLinkProgram(prog)
    gles.glUseProgram(prog)

    W, H = 640, 480
    tex = ctypes.c_uint()
    gles.glGenTextures(1, ctypes.byref(tex))
    gles.glBindTexture(GL_TEXTURE_2D, tex)
    gles.glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)
    gles.glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST)
    gles.glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE)
    gles.glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE)
    gles.glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, W, H, 0, GL_RGBA, GL_UNSIGNED_BYTE, None)

    fbo = ctypes.c_uint()
    gles.glGenFramebuffers(1, ctypes.byref(fbo))
    gles.glBindFramebuffer(GL_FRAMEBUFFER, fbo)
    gles.glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, tex, 0)
    if gles.glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE:
        print("FAIL: FBO incomplete")
        sys.exit(1)

    quad = (ctypes.c_float * 8)(-1, -1, 1, -1, -1, 1, 1, 1)
    gles.glViewport(0, 0, W, H)
    gles.glVertexAttribPointer(0, 2, GL_FLOAT, 0, 0, quad)
    gles.glEnableVertexAttribArray(0)

    pixels = (ctypes.c_ubyte * (W * H * 4))()

    def one_pass():
        t0 = time.monotonic()
        gles.glDrawArrays(GL_TRIANGLE_STRIP, 0, 4)
        gles.glReadPixels(0, 0, W, H, GL_RGBA, GL_UNSIGNED_BYTE, pixels)
        gles.glFinish()
        return (time.monotonic() - t0) * 1000.0

    dt_ms = one_pass()          # first pass: includes JIT/shader-compile/DRI warm-up
    report("post-fbo-pass (first, cold)")
    print(f"  first (cold) {W}x{H} draw+readback pass: {dt_ms:.2f} ms")
    ok = pixels[0] == 255 and pixels[1] == 0
    print(f"  readback sanity check (expect red pixel): {'OK' if ok else 'MISMATCH'}")

    warm_times = [one_pass() for _ in range(20)]
    report("post-warm-passes")
    print(f"  20 warm passes @{W}x{H}: min={min(warm_times):.2f}ms "
          f"avg={sum(warm_times)/len(warm_times):.2f}ms max={max(warm_times):.2f}ms")

    # ---- isolate: is the cost the draw call, or the readback size? ----
    def draw_only():
        t0 = time.monotonic()
        gles.glDrawArrays(GL_TRIANGLE_STRIP, 0, 4)
        gles.glFinish()
        return (time.monotonic() - t0) * 1000.0

    def readback_only(w, h):
        buf = (ctypes.c_ubyte * (w * h * 4))()
        t0 = time.monotonic()
        gles.glReadPixels(0, 0, w, h, GL_RGBA, GL_UNSIGNED_BYTE, buf)
        gles.glFinish()
        return (time.monotonic() - t0) * 1000.0

    _ = [draw_only() for _ in range(3)]           # warm up this specific call path
    draw_times = [draw_only() for _ in range(10)]
    full_rb_times = [readback_only(W, H) for _ in range(10)]
    small_rb_times = [readback_only(4, 4) for _ in range(10)]
    print(f"  draw-only (no readback):     avg={sum(draw_times)/len(draw_times):.2f}ms")
    print(f"  readback-only {W}x{H}:       avg={sum(full_rb_times)/len(full_rb_times):.2f}ms")
    print(f"  readback-only 4x4 (tiny):    avg={sum(small_rb_times)/len(small_rb_times):.2f}ms")

    # ---- a realistic 4-stage downsample chain: 640x480 -> 160x120 -> 40x30 -> 10x8 ->
    # 1x1, mirroring the planned "reduce on GPU, read back a few bytes" design, timed
    # end-to-end including the final tiny readback. ----
    def make_fbo(w, h):
        t = ctypes.c_uint(); f = ctypes.c_uint()
        gles.glGenTextures(1, ctypes.byref(t))
        gles.glBindTexture(GL_TEXTURE_2D, t)
        gles.glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)
        gles.glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST)
        gles.glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE)
        gles.glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE)
        gles.glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, None)
        gles.glGenFramebuffers(1, ctypes.byref(f))
        gles.glBindFramebuffer(GL_FRAMEBUFFER, f)
        gles.glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, t, 0)
        return t, f

    stages = [(160, 120), (40, 30), (10, 8), (1, 1)]
    stage_fbos = [make_fbo(w, h) for w, h in stages]
    tiny_buf = (ctypes.c_ubyte * 4)()

    def reduce_chain():
        t0 = time.monotonic()
        gles.glBindFramebuffer(GL_FRAMEBUFFER, fbo)   # re-render the 640x480 source
        gles.glViewport(0, 0, W, H)
        gles.glDrawArrays(GL_TRIANGLE_STRIP, 0, 4)
        for (w, h), (_, f) in zip(stages, stage_fbos):
            gles.glBindFramebuffer(GL_FRAMEBUFFER, f)
            gles.glViewport(0, 0, w, h)
            gles.glDrawArrays(GL_TRIANGLE_STRIP, 0, 4)   # stand-in for a downsample shader
        gles.glReadPixels(0, 0, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, tiny_buf)
        gles.glFinish()
        return (time.monotonic() - t0) * 1000.0

    _ = [reduce_chain() for _ in range(3)]
    chain_times = [reduce_chain() for _ in range(10)]
    print(f"  full reduce chain 640x480->1x1 (5 passes + tiny readback): "
          f"avg={sum(chain_times)/len(chain_times):.2f}ms "
          f"min={min(chain_times):.2f}ms max={max(chain_times):.2f}ms")

    print("\n=== SUMMARY ===")
    print(f"EGL_EXT_image_dma_buf_import : {has_dmabuf}")
    print(f"GL_OES_EGL_image_external    : {has_ext_image}")
    print(f"cold pass timing @640x480    : {dt_ms:.2f} ms")
    print(f"warm pass timing @640x480    : min={min(warm_times):.2f}ms "
          f"avg={sum(warm_times)/len(warm_times):.2f}ms")
    print(f"final VmRSS                  : {rss_mb():.1f} MB")
    if has_dmabuf and has_ext_image:
        print("-> zero-copy DMA-buf import path looks VIABLE on this driver.")
    else:
        print("-> zero-copy extensions missing/partial -> use plain glTexImage2D upload "
              "of raw YUYV (still decode-free, not copy-free).")


if __name__ == "__main__":
    main()
