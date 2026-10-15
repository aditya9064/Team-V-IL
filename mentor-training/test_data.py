"""Regression checks for training-critical transformations and scoring."""
import tempfile
import unittest
from pathlib import Path
import torch
from detector_data import read_boxes, flip_boxes, crop_to_full
from metrics import coco_metrics, evaluate

class DataTests(unittest.TestCase):
    def test_class_mapping_and_empty_targets(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'label.txt'
            path.write_text('0 .5 .5 .1 .2\n1 .3 .4 .1 .1\n2 .8 .8 .1 .1\n')
            boxes, labels=read_boxes(path,100,50)
            self.assertEqual(labels.tolist(),[1,2,3])
            torch.testing.assert_close(boxes[0],torch.tensor([45.,20.,55.,30.]))
            path.write_text('')
            boxes,labels=read_boxes(path,100,50)
            self.assertEqual(tuple(boxes.shape),(0,4))
            self.assertEqual(tuple(labels.shape),(0,))
            self.assertEqual(labels.dtype,torch.int64)
            self.assertEqual(tuple(flip_boxes(boxes,100).shape),(0,4))

    def test_flip_and_crop_inverse(self):
        boxes=torch.tensor([[10.,5.,30.,25.]])
        flipped=flip_boxes(boxes,100)
        torch.testing.assert_close(flipped,torch.tensor([[70.,5.,90.,25.]]))
        torch.testing.assert_close(flip_boxes(flipped,100),boxes)
        full=crop_to_full(boxes,{'crop_rect_px':[960,1080,2880,1944]})
        torch.testing.assert_close(full,torch.tensor([[970.,1085.,990.,1105.]]))

    def test_invalid_labels_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'label.txt'
            for row in ('3 .5 .5 .1 .1','0 .01 .5 .5 .5', '0 nan .5 .1 .1', '0 .5 .5 0 .1'):
                path.write_text(row)
                with self.assertRaises(ValueError):read_boxes(path,100,50)

    def test_coco_perfect_and_empty_predictions(self):
        gt=dict(info={},images=[dict(id=0,width=100,height=100),dict(id=1,width=100,height=100)],
                categories=[dict(id=i,name=str(i)) for i in (1,2,3)],
                annotations=[dict(id=1,image_id=0,category_id=1,bbox=[10,10,10,10],area=100,iscrowd=0)])
        perfect=[dict(image_id=0,category_id=1,bbox=[10,10,10,10],score=1.)]
        self.assertAlmostEqual(coco_metrics(gt,perfect)['AP50'],1.)
        self.assertEqual(coco_metrics(gt,[])['AP50'],0.)

    def test_evaluator_includes_empty_scene_false_positives(self):
        class Fixture:
            augment=False
            records=[dict(crop_resolution=[20,20],image_path='positive.png',cube_count=1,source_family='positive'),
                     dict(crop_resolution=[20,20],image_path='empty.png',cube_count=0,source_family='empty')]
            def __len__(self):return 2
            def __getitem__(self,i):
                boxes=torch.tensor([[5.,5.,10.,10.]]) if i==0 else torch.empty((0,4))
                labels=torch.tensor([1]) if i==0 else torch.empty((0,),dtype=torch.int64)
                return torch.zeros((3,20,20)),dict(boxes=boxes,labels=labels,image_id=torch.tensor(i))
        class Predictions(torch.nn.Module):
            def forward(self,images):
                return [dict(boxes=torch.tensor([[5.,5.,10.,10.]]),labels=torch.tensor([1]),
                             scores=torch.tensor([score])) for score in (1.,.8)]
        with tempfile.TemporaryDirectory() as directory:
            result=evaluate(Predictions(),Fixture(),'cpu',batch_size=2,output=directory)
            self.assertEqual(result['empty_scene_false_positives_at_0_5'],1)
            self.assertEqual(result['empty_scene_count'],1)
            self.assertAlmostEqual(result['by_source']['positive']['AP50'],1.)
            self.assertIsNone(result['by_source']['empty']['AP50'])
            self.assertTrue((Path(directory)/'predictions.json').is_file())

if __name__=='__main__':unittest.main()
