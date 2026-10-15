"""Make review sheets from already verified diversity/legacy labels."""
import argparse
import json
from pathlib import Path
from PIL import Image,ImageDraw
from varied_labels import project,rotate

def main():
    p=argparse.ArgumentParser();p.add_argument('--folders',type=Path,nargs='+',required=True)
    p.add_argument('--out',type=Path,required=True);a=p.parse_args();a.out.mkdir(parents=True,exist_ok=True)
    items=[];montage=Image.new('RGB',(1280,390*((len(a.folders)+1)//2)),'white')
    draw=ImageDraw.Draw(montage)
    for index,folder in enumerate(a.folders):
        annotations=json.loads((folder/'annotation_qa.json').read_text())
        assert annotations['passed']
        camera=json.loads((folder/'camera.json').read_text())
        state=json.loads((folder/'scene_state.json').read_text())
        with Image.open(folder/'labels_overlay.png') as overlay:
            preview=overlay.convert('RGB');preview.thumbnail((640,360))
            x,y=(index%2)*640,(index//2)*390
            montage.paste(preview,(x,y+30));draw.text((x+5,y+7),f"{folder.name} — {annotations['cube_count']} cubes",fill='black')
        with Image.open(folder/'world_camera.png') as image:
            image.load()
            for obj in annotations['objects']:
                pose=state['objects'][obj['name']]
                import numpy as np
                centre=np.array(pose['position'])+rotate(np.array([0,0,.005]),pose['orientation'])
                u,v=map(round,project(camera,centre))
                items.append((f"{folder.name} {obj['colour']}",image.crop((u-40,v-40,u+40,v+40))))
    montage.save(a.out/'scenes-label-review.png')
    for start in range(0,len(items),60):
        batch=items[start:start+60];sheet=Image.new('RGB',(960,110*((len(batch)+5)//6)),'white')
        draw=ImageDraw.Draw(sheet)
        for index,(name,patch) in enumerate(batch):
            x,y=(index%6)*160,(index//6)*110
            sheet.paste(patch,(x+40,y+25));draw.text((x+5,y+5),name,fill='black')
        sheet.save(a.out/f'cube-patches-{start//60+1:02}.png')
    print('Review sheets:',len(a.folders),'scenes,',len(items),'cube patches')

if __name__=='__main__':main()
