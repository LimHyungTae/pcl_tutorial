#!/usr/bin/env python3
"""Generate the social-share cover image at web/public/og-cover.png.

1200 x 630 is the canonical OpenGraph dimension. We render:
  - dark navy background
  - randomly placed soft point-cloud dots (mimics the demo's aesthetic)
  - title "Interactive PCL Tutorial"
  - subtitle "Hands-on Point Cloud Library · PCL 튜토리얼"
  - URL footer

Re-run only when the design needs to change; the PNG itself is checked
in so a clean clone has it without needing Pillow.
"""
import random
from pathlib import Path

from PIL import Image, ImageDraw, ImageFilter, ImageFont

REPO = Path(__file__).resolve().parent.parent
OUT = REPO / "web" / "public" / "og-cover.png"

W, H = 1200, 630
BG = (10, 15, 26)           # matches --bg in the app theme
ACCENT = (0, 212, 170)      # --accent (teal)
ACCENT_2 = (77, 166, 255)   # --accent-2 (blue)
TEXT = (236, 241, 249)
DIM = (160, 178, 199)
MUTED = (110, 128, 150)

random.seed(7)  # deterministic so commits stay clean


def _resolve_font(
    size: int, bold: bool = False, mono: bool = False, cjk: bool = False
) -> ImageFont.FreeTypeFont:
    """Best-effort font resolution that works on macOS, Linux, and CI.

    cjk=True picks a font that covers Hangul + CJK brackets; the Latin
    fallbacks don't have those glyphs so they render as tofu.
    """
    candidates: list[str] = []
    if cjk:
        candidates += [
            "/System/Library/Fonts/AppleSDGothicNeo.ttc",
            "/System/Library/Fonts/Hiragino Sans GB.ttc",
            "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
            "/usr/share/fonts/truetype/noto/NotoSansCJK-Regular.ttc",
        ]
    elif mono:
        candidates += [
            "/System/Library/Fonts/Menlo.ttc",
            "/System/Library/Fonts/Monaco.dfont",
            "/usr/share/fonts/truetype/dejavu/DejaVuSansMono-Bold.ttf",
            "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf",
        ]
    elif bold:
        candidates += [
            "/System/Library/Fonts/Supplemental/Arial Bold.ttf",
            "/System/Library/Fonts/HelveticaNeue.ttc",
            "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
            "/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf",
        ]
    else:
        candidates += [
            "/System/Library/Fonts/Helvetica.ttc",
            "/System/Library/Fonts/Supplemental/Arial.ttf",
            "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
            "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf",
        ]
    for path in candidates:
        if Path(path).exists():
            try:
                return ImageFont.truetype(path, size)
            except OSError:
                continue
    return ImageFont.load_default()


def _draw_dots(draw: ImageDraw.ImageDraw) -> None:
    """Scatter ~600 soft dots in two color bands to evoke a point cloud."""
    for _ in range(620):
        x = random.gauss(W * 0.78, 180)
        y = random.gauss(H * 0.55, 150)
        if not (0 <= x < W and 0 <= y < H):
            continue
        r = random.choice([1, 1, 1, 2, 2, 3])
        # Closer to the focal point → brighter, far away → dim.
        d = ((x - W * 0.78) ** 2 + (y - H * 0.55) ** 2) ** 0.5
        intensity = max(0.0, 1.0 - d / 320.0)
        color = ACCENT if random.random() < 0.6 else ACCENT_2
        c = tuple(int(BG[i] + (color[i] - BG[i]) * (0.4 + 0.6 * intensity)) for i in range(3))
        draw.ellipse([x - r, y - r, x + r, y + r], fill=c)


def main() -> None:
    img = Image.new("RGB", (W, H), BG)

    # Dot field on a separate layer, blurred to look like depth.
    dots = Image.new("RGB", (W, H), BG)
    _draw_dots(ImageDraw.Draw(dots))
    dots = dots.filter(ImageFilter.GaussianBlur(radius=0.6))
    img = Image.blend(img, dots, 0.95)

    draw = ImageDraw.Draw(img)

    # Accent bar on the left.
    draw.rectangle([0, 0, 8, H], fill=ACCENT)

    # Top tag — needs CJK font for 【 】.
    tag_font = _resolve_font(30, cjk=True)
    draw.text((72, 68), "【Interactive】", font=tag_font, fill=ACCENT)

    # Title (two lines so 'PCL Tutorial' is huge and readable).
    title_font = _resolve_font(112, bold=True)
    draw.text((72, 130), "PCL Tutorial", font=title_font, fill=TEXT)

    sub_font = _resolve_font(40, bold=False)
    draw.text((72, 280), "Hands-on Point Cloud Library", font=sub_font, fill=DIM)

    # Korean alt subtitle so the cover signals bilingual content.
    ko_font = _resolve_font(28, cjk=True)
    draw.text((72, 340), "PCL 튜토리얼 · 브라우저에서 바로 실습", font=ko_font, fill=MUTED)

    # Feature chips, monospaced like the in-app code-font.
    chip_font = _resolve_font(22, mono=True)
    chips = ["voxel", "KdTree", "ICP", "RANSAC", "KISS-Matcher", "GenZ-ICP"]
    x = 72
    y = 430
    for c in chips:
        w = draw.textlength(c, font=chip_font)
        pad_x, pad_y = 14, 8
        box = [x, y, x + w + pad_x * 2, y + 22 + pad_y * 2]
        draw.rounded_rectangle(box, radius=10, outline=ACCENT, width=1)
        draw.text((x + pad_x, y + pad_y - 2), c, font=chip_font, fill=ACCENT)
        x += w + pad_x * 2 + 12

    # Footer.
    foot_font = _resolve_font(22, mono=True)
    draw.text(
        (72, H - 60),
        "limhyungtae.github.io/pcl_tutorial",
        font=foot_font,
        fill=DIM,
    )

    OUT.parent.mkdir(parents=True, exist_ok=True)
    img.save(OUT, "PNG", optimize=True)
    print(f"wrote {OUT.relative_to(REPO)} ({OUT.stat().st_size / 1024:.1f} KB)")


if __name__ == "__main__":
    main()
