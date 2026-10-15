"""Regression for the exporter defect: cube roots are at their bottom face."""
import unittest
import numpy as np
from varied_labels import cube_hull

class BottomOriginRegression(unittest.TestCase):
    def test_projected_extent_includes_cube_above_root(self):
        camera=dict(resolution=[1000,1000],camera_to_world=np.eye(4).tolist(),
            focal_length_mm=1,horizontal_aperture_mm=2,vertical_aperture_mm=2)
        pose=dict(position=[0,0,-5],orientation=[1,0,0,0])
        hull=cube_hull(camera,pose,1.)
        # Nearest vertices lie at z=-4, not -4.5. A centre-origin exporter
        # gives the wrong bounds and fails this independently derived fixture.
        np.testing.assert_allclose(hull.min(0),[437.5,437.5])
        np.testing.assert_allclose(hull.max(0),[562.5,562.5])

if __name__=='__main__':unittest.main()
