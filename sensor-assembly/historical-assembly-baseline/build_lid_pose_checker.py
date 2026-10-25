from pathlib import Path
import json
p=Path(__file__).parent;s=(p/'check_gentle_lid_full.py').read_text()
for a,b in json.loads((p/'lid-pose-checker-patches.json').read_text()):
 assert s.count(a)==1,(a[:90],s.count(a));s=s.replace(a,b)
(p/'check_lid_pose.py').write_text(s)
