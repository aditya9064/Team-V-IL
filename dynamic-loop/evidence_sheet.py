"""Make a review sheet from recorded camera evidence; never simulator truth."""
import argparse
import json
from pathlib import Path
from PIL import Image, ImageDraw

p=argparse.ArgumentParser()
p.add_argument('run',type=Path)
a=p.parse_args()
r=json.loads((a.run/'execution_report.json').read_text())
folders={int(f.name.split('-')[0]):f for f in (a.run/'observations').iterdir() if f.is_dir()}
rows=[]
for attempt in r['attempts']:
    pickup=attempt.get('pickup_evidence',[])
    if not pickup:continue
    proof=pickup[-1];lift_id=proof['observation_id']
    source_id=attempt['source_observation_id']
    refs=[(i,f) for i,f in folders.items() if source_id<i<lift_id and f.name.endswith('empty-hand-closed')]
    if not refs:continue
    _,ref=max(refs)
    side=proof['views']['side']
    if 'box' not in side:continue
    x0,y0,x1,y1=side['box'];pad=12
    box=(x0-pad,y0-pad,x1+pad,y1+pad)
    source=Image.open(folders[source_id]/'table.png').convert('RGB')
    u,v=attempt['target']['pixel_xy']
    source=source.crop((int(u)-45,int(v)-45,int(u)+45,int(v)+45))
    empty=Image.open(ref/'side.png').convert('RGB').crop(box)
    held=Image.open(folders[lift_id]/'side.png').convert('RGB').crop(box)
    placement=attempt.get('placement_evidence')
    images=[source,empty,held]
    texts=['Source: '+attempt['target']['colour'],'Empty hand reference','Lift: '+proof['verdict']]
    if placement and placement['side'] and 'box' in placement['side'][-1]:
        dest=placement['side'][-1];dest_id=placement['observation_ids'][-1]
        x0,y0,x1,y1=dest['box']
        images.append(Image.open(folders[dest_id]/'side.png').convert('RGB').crop((x0-pad,y0-pad,x1+pad,y1+pad)))
        texts.append('Placement: '+str(attempt['placement_verified']))
    rows.append((attempt,images,texts))
if not rows:raise SystemExit('No attempted pickup camera evidence yet')
width=1200;height=65+len(rows)*245
sheet=Image.new('RGB',(width,height),'white');draw=ImageDraw.Draw(sheet)
draw.text((10,10),f"{a.run.name} | {r['status']} | camera evidence, enlarged crops",fill='black')
draw.text((10,30),'Source -> empty-hand reference -> lift observation -> destination observation. Full frames and metadata are retained.',fill='black')
for row,(attempt,images,texts) in enumerate(rows):
    top=65+row*245
    for col,(img,label) in enumerate(zip(images,texts)):
        img.thumbnail((280,205),Image.Resampling.NEAREST)
        # Enlarge without inventing image detail; these are diagnostic pixels.
        scale=min(280/img.width,205/img.height)
        img=img.resize((round(img.width*scale),round(img.height*scale)),Image.Resampling.NEAREST)
        sheet.paste(img,(col*300+(300-img.width)//2,top+25))
        draw.text((col*300+10,top),f"Attempt {attempt['number']}: {label}",fill='black')
out=a.run/'camera-evidence.jpg';sheet.save(out,quality=92)
print(out)
