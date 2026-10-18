from PIL import Image, ImageDraw

import transfer_qc as qc

SIZE = (200, 100)
BOX = (0.25, 0.5, 0.2, 0.4)  # cx, cy, w, h -> pixels 30..70 x 30..70


def scene(cube_rgb, extra=None):
    image = Image.new("RGB", SIZE, (235, 235, 235))
    draw = ImageDraw.Draw(image)
    draw.rectangle([30, 30, 70, 70], fill=cube_rgb)
    if extra:
        draw.rectangle(extra[0], fill=extra[1])
    return image


def write(tmp_path, original, generated):
    paths = tmp_path / "o.png", tmp_path / "g.jpg", tmp_path / "o.txt"
    original.save(paths[0])
    generated.save(paths[1], quality=95)
    paths[2].write_text("0 0.25 0.5 0.2 0.4\n")
    return paths


def test_colour_name_bins():
    assert qc.colour_name(220, 30, 30) == "red"
    assert qc.colour_name(230, 210, 20) == "yellow"
    assert qc.colour_name(30, 80, 220) == "blue"
    assert qc.colour_name(200, 200, 200) == "grey"


def test_same_colour_passes(tmp_path):
    red = (220, 30, 30)
    result = qc.check(*write(tmp_path, scene(red), scene((200, 40, 35))))
    assert (result["kept"], result["changed"], result["pass"]) == (1, 0, True)


def test_colour_flip_fails(tmp_path):
    result = qc.check(*write(tmp_path, scene((220, 30, 30)), scene((30, 80, 220))))
    assert (result["kept"], result["changed"], result["pass"]) == (0, 1, False)


def test_hallucinated_cube_outside_boxes_fails(tmp_path):
    red = (220, 30, 30)
    generated = scene(red, extra=([140, 30, 180, 70], (30, 80, 220)))
    result = qc.check(*write(tmp_path, scene(red), generated))
    assert result["changed"] == 0
    assert result["new_colour_fraction"] > qc.MAX_NEW_COLOUR_FRACTION
    assert not result["pass"]


def test_resized_output_is_compared_at_original_size(tmp_path):
    red = (220, 30, 30)
    result = qc.check(*write(tmp_path, scene(red), scene(red).resize((400, 200))))
    assert result["pass"]
