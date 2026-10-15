import copy
import math
import unittest
from detector_contract import DetectorContractError,map_predictions,validate_deployment,deployment_id

def config():
    return dict(schema_version=1,architecture='fasterrcnn_resnet50_fpn_v2',classes=['yellow','blue','red'],
                foreground_labels=[1,2,3],full_resolution=[3840,2160],crop_rect_px=[960,1080,2880,1944],
                inference_resolution=[1920,864],score_threshold=.8,threshold_source='unit-test-only',
                checkpoint_sha256='a'*64,min_size=864,max_size=1920)

def output():return dict(boxes=[[10.,20.,30.,40.]],labels=[1],scores=[.9])

class DetectorContractTests(unittest.TestCase):
    def mapped(self,data=None,**changes):
        c=config();c.update(changes)
        return map_predictions(data or output(),c['full_resolution'],c['crop_rect_px'],c['inference_resolution'],c['score_threshold'])
    def test_crop_offset_and_class_background_shift(self):
        accepted,_=self.mapped();p=accepted[0]
        self.assertEqual(p['colour'],'yellow');self.assertEqual(p['bbox_full_px'],[970,1100,990,1120])
        self.assertEqual(p['pixel_xy'],[980,1110])
        data=output();data['labels']=[3];self.assertEqual(self.mapped(data)[0][0]['colour'],'red')
    def test_explicit_resize_restores_full_camera_coordinates(self):
        p=self.mapped(inference_resolution=[960,432])[0][0]
        self.assertEqual(p['bbox_full_px'],[980,1120,1020,1160])
    def test_empty_predictions_are_valid_missing_inventory(self):
        self.assertEqual(self.mapped(dict(boxes=[],labels=[],scores=[])),([],[]))
    def test_low_score_is_rejected_not_relabelled(self):
        d=output();d['scores']=[.79];a,r=self.mapped(d)
        self.assertEqual(a,[]);self.assertEqual(r[0]['rejection_reason'],'score_below_threshold')
    def test_unknown_or_background_labels_fail_closed(self):
        for label in (0,4,True,1.5):
            d=output();d['labels']=[label]
            with self.assertRaises(DetectorContractError):self.mapped(d)
    def test_malformed_shapes_and_nonfinite_values_fail_closed(self):
        bad=[dict(boxes=[],labels=[1],scores=[.9])]
        for key,value in (('scores',[float('nan')]),('scores',[float('inf')]),('scores',[1.1]),
                          ('boxes',[[0,0,float('nan'),4]]),('boxes',[[3,2,1,4]]),('boxes',[[-1,0,3,4]]),
                          ('boxes',[[0,0,1921,4]]),('boxes',[[1,2,3]])):
            d=output();d[key]=value;bad.append(d)
        for d in bad:
            with self.subTest(data=d),self.assertRaises(DetectorContractError):self.mapped(d)
    def test_invalid_mapping_is_not_guessed_or_clipped(self):
        for changes in ({'crop_rect_px':[0,0,4000,2000]},{'inference_resolution':[0,864]},
                        {'score_threshold':float('nan')},{'score_threshold':True}):
            with self.assertRaises(DetectorContractError):self.mapped(**changes)
    def test_deployment_requires_threshold_provenance_and_checkpoint_hash(self):
        validate_deployment(config())
        for key,value in (('score_threshold',None),('threshold_source',''),('checkpoint_sha256',None),
                          ('classes',['red','blue','yellow']),('foreground_labels',[0,1,2])):
            c=config();c[key]=value
            with self.subTest(key=key),self.assertRaises(DetectorContractError):validate_deployment(c)
    def test_identity_pins_weights_and_configuration(self):
        a=config();b=copy.deepcopy(a);b['score_threshold']=.85
        self.assertNotEqual(deployment_id(a),deployment_id(b))
        b=config();b['checkpoint_sha256']='b'*64
        self.assertNotEqual(deployment_id(a),deployment_id(b))

if __name__=='__main__':unittest.main()
