from pathlib import Path
import json
p=Path(__file__).parent;s=(p/'gentle_lid_full_assembly_trial.py').read_text()
for a,b in json.loads((p/'lid-pose-runtime-patches.json').read_text()):
 assert s.count(a)==1,(a[:90],s.count(a));s=s.replace(a,b)
(p/'lid_pose_trial.py').write_text(s)
