"""Analytic pixel geometry and unchanged colour/occlusion rejection gates."""
import os
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

class VisionGeometryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        starter=os.environ.get('FANUC_STARTER')
        if not starter or not Path(starter,'scene_config.py').exists():
            raise unittest.SkipTest('Set FANUC_STARTER to the approved starter directory')
        sys.path.insert(0,starter)
        import numpy as np
        import vision
        cls.np=np;cls.vision=vision

    def test_integer_square_has_geometric_area_not_inclusive_fill_border(self):
        np=self.np
        mask=self.vision.pixel_centre_polygon([[1,1],[4,1],[4,4],[1,4]],(6,6))
        expected=np.zeros((6,6),bool);expected[1:4,1:4]=True
        np.testing.assert_array_equal(mask,expected)
        self.assertEqual(int(mask.sum()),9)

    def test_fractional_rectangle_and_reversed_winding(self):
        np=self.np;points=np.array([[1.2,.2],[3.2,.2],[3.2,2.2],[1.2,2.2]])
        expected=np.zeros((5,5),bool);expected[0:2,1:3]=True
        for hull in (points,points[::-1]):
            np.testing.assert_array_equal(self.vision.pixel_centre_polygon(hull,(5,5)),expected)

    def test_integer_translation_preserves_sampled_shape(self):
        np=self.np;hull=np.array([[.2,.2],[3.2,.2],[.2,3.2]])
        mask=self.vision.pixel_centre_polygon(hull,(5,5))
        translated=self.vision.pixel_centre_polygon(hull+[3,2],(8,9))
        np.testing.assert_array_equal(translated[2:7,3:8],mask)
        self.assertEqual(int(mask.sum()),int(translated.sum()))

    def test_partial_colour_below_65_percent_still_rejected(self):
        np=self.np;image=np.zeros((10,10,3),np.uint8)
        image[:6,:,0]=255 # 60 percent coherent blue; insufficient placement proof.
        with patch.object(self.vision,'cube_region',return_value=([0,0,10,10],np.ones((10,10),bool))):
            result=self.vision.colour_evidence(image,{},[0,0,0],'blue')
        self.assertEqual(result['threshold'],.65)
        self.assertFalse(result['positive'])

    def test_wrong_colour_and_unchanged_reference_still_rejected(self):
        np=self.np;blue=np.zeros((10,10,3),np.uint8);blue[:,:,0]=255
        with patch.object(self.vision,'cube_region',return_value=([0,0,10,10],np.ones((10,10),bool))):
            wrong=self.vision.colour_evidence(blue,{},[0,0,0],'yellow')
            old=self.vision.colour_evidence(blue,{},[0,0,0],'blue',reference=blue)
        self.assertFalse(wrong['positive']);self.assertIn('blue',wrong['wrong_colour'])
        self.assertFalse(old['positive']);self.assertTrue(old['empty_reference_match'])
        self.assertEqual(old['threshold'],.35)

    def test_neural_confidence_cannot_replace_pose_fit_gate(self):
        from types import SimpleNamespace
        np=self.np;image=np.zeros((100,100,3),np.uint8)
        proposal=dict(colour='blue',pixel_xy=[60.,60.],detector_score=.999,bbox_full_px=[50,50,70,70])
        cam=SimpleNamespace(table_z=.9,to_plane=lambda *args,**kwargs:np.array([.32,.1,.905]))
        with patch.object(self.vision,'CameraModel',return_value=cam),patch.object(self.vision,'refine_pose',return_value=(np.array([.32,.1]),0.,.84)):
            accepted,rejected=self.vision.refine_proposals(image,{'resolution':[100,100]},[proposal])
        self.assertEqual(accepted,[]);self.assertEqual(rejected[0]['rejection_reason'],'pose_silhouette_fit_rejected')
    def test_duplicate_model_proposals_after_fitting_hold(self):
        from types import SimpleNamespace
        np=self.np;image=np.zeros((100,100,3),np.uint8)
        proposal=dict(colour='blue',pixel_xy=[60.,60.],detector_score=.99,bbox_full_px=[50,50,70,70])
        cam=SimpleNamespace(table_z=.9,to_plane=lambda *args,**kwargs:np.array([.32,.1,.905]))
        with patch.object(self.vision,'CameraModel',return_value=cam),patch.object(self.vision,'refine_pose',return_value=(np.array([.32,.1]),0.,.95)):
            with self.assertRaisesRegex(ValueError,'Duplicate'):
                self.vision.refine_proposals(image,{'resolution':[100,100]},[proposal,proposal])

if __name__=='__main__':unittest.main()
