from torchvision.models.detection import fasterrcnn_resnet50_fpn_v2, FasterRCNN_ResNet50_FPN_V2_Weights
from torchvision.models.detection.faster_rcnn import FastRCNNPredictor
from torchvision.models.detection.anchor_utils import AnchorGenerator

def make_model(pretrained=True, min_size=864, max_size=1920):
    # Smaller initial anchors for the small cube boxes; tune using validation only.
    anchors = AnchorGenerator(((16,), (32,), (64,), (128,), (256,)),
                              ((.5, 1., 2.),)*5)
    model = fasterrcnn_resnet50_fpn_v2(
        weights=FasterRCNN_ResNet50_FPN_V2_Weights.DEFAULT if pretrained else None,
        weights_backbone=None, min_size=min_size, max_size=max_size)
    # v2 supplies anchors internally; replace them after construction while
    # retaining three anchors per location, so the pretrained RPN head fits.
    if model.rpn.anchor_generator.num_anchors_per_location() != anchors.num_anchors_per_location():
        raise ValueError('Anchor count differs from the pretrained RPN head')
    model.rpn.anchor_generator = anchors
    features = model.roi_heads.box_predictor.cls_score.in_features
    model.roi_heads.box_predictor = FastRCNNPredictor(features, 4)
    return model
