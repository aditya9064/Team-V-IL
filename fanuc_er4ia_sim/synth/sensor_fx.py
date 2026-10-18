"""
Camera imperfections applied after rendering (v2): exposure, gamma, white
balance, vignetting, sensor noise, blur and JPEG compression.

Everything here is pixel-local or a small blur, so object boxes and labels
stay valid. Parameters come from scenarios._sensor; the noise draw is seeded
per frame so a frame is reproducible.
"""

import cv2
import numpy as np


def _tone(img, gain, gamma, wb):
    x = img.astype(np.float32) / 255.0
    x = x * gain * np.array(wb[::-1], np.float32)  # wb is RGB; image is BGR
    x = np.clip(x, 0, 1) ** gamma
    return x


def _vignette(x, strength):
    h, w = x.shape[:2]
    yy, xx = np.mgrid[0:h, 0:w].astype(np.float32)
    r2 = ((xx - w / 2) / (w / 2)) ** 2 + ((yy - h / 2) / (h / 2)) ** 2
    return x * (1 - strength * np.clip(r2 / 2, 0, 1))[..., None]


def _blur(img, kind, sigma, length, angle):
    if kind == "gaussian":
        return cv2.GaussianBlur(img, (0, 0), sigma)
    if kind == "motion":
        k = np.zeros((length, length), np.float32)
        k[length // 2, :] = 1.0
        rot = cv2.getRotationMatrix2D((length / 2 - 0.5, length / 2 - 0.5), angle, 1.0)
        k = cv2.warpAffine(k, rot, (length, length))
        return cv2.filter2D(img, -1, k / max(k.sum(), 1e-6))
    return img


def apply(img, params, seed):
    """BGR uint8 in, BGR uint8 out."""
    rng = np.random.default_rng(seed)
    x = _vignette(_tone(img, params["gain"], params["gamma"], params["wb"]), params["vignette"])
    out = np.clip(np.rint(x * 255.0), 0, 255).astype(np.uint8)
    out = _blur(out, params["blur"], params["blur_sigma"], params["motion_len"], params["motion_angle"])
    if params["noise"] > 0:
        noisy = out.astype(np.float32) + rng.normal(0, params["noise"], out.shape)
        out = np.clip(noisy, 0, 255).astype(np.uint8)
    if params["jpeg"]:
        ok, buf = cv2.imencode(".jpg", out, [cv2.IMWRITE_JPEG_QUALITY, params["jpeg"]])
        out = cv2.imdecode(buf, cv2.IMREAD_COLOR) if ok else out
    return out


IDENTITY = {"gain": 1.0, "gamma": 1.0, "wb": (1.0, 1.0, 1.0), "noise": 0.0, "blur": "none",
            "blur_sigma": 0.0, "motion_len": 3, "motion_angle": 0.0, "jpeg": 0, "vignette": 0.0}
