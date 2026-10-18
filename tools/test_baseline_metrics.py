import baseline_metrics as bm


def test_yolo_label_boxes_converts_centre_format():
    assert bm.yolo_label_boxes("2 0.5 0.5 0.2 0.4\n") == [(2, (0.4, 0.3, 0.6, 0.7))]


def test_cosmos_boxes_maps_categories_and_colours():
    parsed = {"objects": [
        {"category": "cube", "colour": "Blue", "bbox": [0.1, 0.1, 0.2, 0.2]},
        {"category": "cube", "colour": "green", "bbox": [0.1, 0.1, 0.2, 0.2]},
        {"category": "vial", "bbox": [0.3, 0.3, 0.4, 0.4]},
        {"category": "fixture", "bbox": [0.1, 0.1, 0.2, 0.2]},
        {"category": "cap", "bbox": [0.5, 0.5, 0.4, 0.6]},
        {"category": "cap", "bbox": ["x", 0, 1, 1]},
    ]}
    assert [c for c, _ in bm.cosmos_boxes(parsed)] == [2, bm.WRONG_CLASS, 4]


def test_cosmos_boxes_handles_unparsed_reply():
    assert bm.cosmos_boxes(None) == []


def test_match_counts_class_errors_only_when_class_used():
    truths = [(0, (0.1, 0.1, 0.2, 0.2))]
    preds = [(1, (0.1, 0.1, 0.2, 0.2))]
    assert bm.match(preds, truths) == (0, 1, 1)
    assert bm.match(preds, truths, use_class=False) == (1, 0, 0)


def test_match_is_one_to_one():
    truths = [(3, (0.0, 0.0, 0.5, 0.5))]
    preds = [(3, (0.0, 0.0, 0.5, 0.5)), (3, (0.0, 0.0, 0.5, 0.5))]
    assert bm.match(preds, truths) == (1, 1, 0)


def test_match_rejects_low_overlap():
    assert bm.match([(0, (0.0, 0.0, 0.1, 0.1))], [(0, (0.05, 0.05, 0.2, 0.2))]) == (0, 1, 1)


def test_summarise_groups_by_set_camera_and_all():
    row = {"detector": "yolo", "set": "v2_hard", "camera": "side", "tp": 3, "fp": 1, "fn": 1,
           "loc_tp": 4, "seconds": 0.5, "parsed": True}
    out = bm.summarise([row, dict(row, camera="wrist", parsed=False)])
    assert out[("yolo", "ALL")]["images"] == 2
    assert out[("yolo", "ALL")]["precision"] == 0.75
    assert out[("yolo", "ALL")]["unparsed"] == 1
    assert out[("yolo", "v2_hard/side")]["found_any_class"] == 1.0
