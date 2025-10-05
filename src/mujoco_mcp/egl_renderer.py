"""
EGL Headless Rendering for MuJoCo
Enables GPU rendering in server environments without display
"""

import os
import logging
from typing import Optional, Dict, Any, Tuple
import numpy as np

try:
    import OpenGL.EGL as egl
    import OpenGL.GL as gl

    EGL_AVAILABLE = True
except (ImportError, AttributeError) as e:
    EGL_AVAILABLE = False
    _egl_import_error = str(e)

import mujoco

logger = logging.getLogger(__name__)


class EGLContext:
    """EGL context manager for headless GPU rendering"""

    def __init__(self, width: int = 640, height: int = 480):
        self.width = width
        self.height = height
        self.display = None
        self.context = None
        self.surface = None
        self._initialized = False

    def __enter__(self):
        """Initialize EGL context"""
        if not EGL_AVAILABLE:
            raise RuntimeError("EGL not available. Install PyOpenGL package.")

        try:
            # Get EGL display
            self.display = egl.eglGetDisplay(egl.EGL_DEFAULT_DISPLAY)
            if self.display == egl.EGL_NO_DISPLAY:
                raise RuntimeError("Failed to get EGL display")

            # Initialize EGL
            major, minor = egl.eglInitialize(self.display, None, None)
            logger.info(f"EGL initialized: version {major}.{minor}")

            # Configure EGL
            config_attribs = [
                egl.EGL_SURFACE_TYPE,
                egl.EGL_PBUFFER_BIT,
                egl.EGL_BLUE_SIZE,
                8,
                egl.EGL_GREEN_SIZE,
                8,
                egl.EGL_RED_SIZE,
                8,
                egl.EGL_DEPTH_SIZE,
                24,
                egl.EGL_RENDERABLE_TYPE,
                egl.EGL_OPENGL_BIT,
                egl.EGL_COLOR_BUFFER_TYPE,
                egl.EGL_RGB_BUFFER,
                egl.EGL_NONE,
            ]

            configs = egl.eglChooseConfig(self.display, config_attribs, 1)
            if not configs:
                raise RuntimeError("Failed to choose EGL config")
            config = configs[0]

            # Create EGL context
            context_attribs = [
                egl.EGL_CONTEXT_MAJOR_VERSION,
                3,
                egl.EGL_CONTEXT_MINOR_VERSION,
                3,
                egl.EGL_NONE,
            ]

            egl.eglBindAPI(egl.EGL_OPENGL_API)
            self.context = egl.eglCreateContext(
                self.display, config, egl.EGL_NO_CONTEXT, context_attribs
            )
            if self.context == egl.EGL_NO_CONTEXT:
                raise RuntimeError("Failed to create EGL context")

            # Create pbuffer surface
            pbuffer_attribs = [egl.EGL_WIDTH, self.width, egl.EGL_HEIGHT, self.height, egl.EGL_NONE]

            self.surface = egl.eglCreatePbufferSurface(self.display, config, pbuffer_attribs)
            if self.surface == egl.EGL_NO_SURFACE:
                raise RuntimeError("Failed to create EGL surface")

            # Make context current
            if not egl.eglMakeCurrent(self.display, self.surface, self.surface, self.context):
                raise RuntimeError("Failed to make EGL context current")

            self._initialized = True
            logger.info(f"EGL context initialized: {self.width}x{self.height}")
            return self

        except Exception as e:
            self.__exit__(None, None, None)
            raise RuntimeError(f"Failed to initialize EGL context: {e}")

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Cleanup EGL context"""
        try:
            if self.context:
                egl.eglMakeCurrent(
                    self.display, egl.EGL_NO_SURFACE, egl.EGL_NO_SURFACE, egl.EGL_NO_CONTEXT
                )
                egl.eglDestroyContext(self.display, self.context)
                self.context = None

            if self.surface:
                egl.eglDestroySurface(self.display, self.surface)
                self.surface = None

            if self.display:
                egl.eglTerminate(self.display)
                self.display = None

            self._initialized = False
            logger.info("EGL context cleaned up")

        except Exception as e:
            logger.warning(f"Error during EGL cleanup: {e}")

    def is_initialized(self) -> bool:
        """Check if EGL context is initialized"""
        return self._initialized


class EGLRenderer:
    """EGL-based headless renderer for MuJoCo"""

    def __init__(self, model: mujoco.MjModel, width: int = 640, height: int = 480):
        self.model = model
        self.width = width
        self.height = height
        self._egl_context = None
        self._renderer = None

    def __enter__(self):
        """Initialize EGL renderer"""
        self._egl_context = EGLContext(self.width, self.height)
        self._egl_context.__enter__()

        # Create MuJoCo renderer within EGL context
        self._renderer = mujoco.Renderer(self.model, self.height, self.width)
        logger.info(f"EGL renderer initialized: {self.width}x{self.height}")
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Cleanup EGL renderer"""
        if self._renderer:
            self._renderer = None

        if self._egl_context:
            self._egl_context.__exit__(exc_type, exc_val, exc_tb)
            self._egl_context = None

    def render(self, data: mujoco.MjData, camera_id: int = -1) -> np.ndarray:
        """Render frame using EGL"""
        if not self._renderer or not self._egl_context.is_initialized():
            raise RuntimeError("EGL renderer not initialized")

        try:
            # Update scene and render
            self._renderer.update_scene(data, camera=camera_id)
            pixels = self._renderer.render()

            # Convert to standard RGB format
            if pixels.shape[2] == 4:  # RGBA to RGB
                pixels = pixels[:, :, :3]

            return pixels

        except Exception as e:
            logger.error(f"EGL rendering failed: {e}")
            raise


def check_egl_support() -> Dict[str, Any]:
    """Check EGL support and capabilities"""
    info = {"egl_available": EGL_AVAILABLE, "gpu_available": False, "error": None}

    if not EGL_AVAILABLE:
        error_msg = "EGL not available"
        if "_egl_import_error" in globals():
            error_msg += f" - {_egl_import_error}"
        else:
            error_msg += " - install PyOpenGL"
        info["error"] = error_msg
        return info

    try:
        # Try to get EGL display
        display = egl.eglGetDisplay(egl.EGL_DEFAULT_DISPLAY)
        if display == egl.EGL_NO_DISPLAY:
            info["error"] = "No EGL display available"
            return info

        # Try to initialize EGL
        major_version = None
        minor_version = None
        try:
            major, minor = egl.eglInitialize(display, None, None)
            major_version = major
            minor_version = minor
        except Exception as init_error:
            info["error"] = f"EGL initialization failed: {init_error}"
            return info

        # If we get here, EGL is working
        with EGLContext(64, 64):
            info["gpu_available"] = True

            # Get GPU info
            try:
                vendor = gl.glGetString(gl.GL_VENDOR)
                renderer = gl.glGetString(gl.GL_RENDERER)
                version = gl.glGetString(gl.GL_VERSION)

                info.update(
                    {
                        "vendor": vendor.decode("utf-8") if vendor else "Unknown",
                        "renderer": renderer.decode("utf-8") if renderer else "Unknown",
                        "version": version.decode("utf-8") if version else "Unknown",
                        "egl_version": (
                            f"{major_version}.{minor_version}" if major_version else "Unknown"
                        ),
                    }
                )
            except Exception as gl_error:
                info["error"] = f"OpenGL query failed: {gl_error}"

        # Clean up
        try:
            egl.eglTerminate(display)
        except:
            pass

    except Exception as e:
        info["error"] = str(e)

    return info


def create_egl_renderer(
    model: mujoco.MjModel, width: int = 640, height: int = 480
) -> Optional[EGLRenderer]:
    """Factory function to create EGL renderer with fallback"""
    if not EGL_AVAILABLE:
        logger.warning("EGL not available, falling back to software rendering")
        return None

    try:
        return EGLRenderer(model, width, height)
    except Exception as e:
        logger.warning(f"Failed to create EGL renderer: {e}")
        return None
