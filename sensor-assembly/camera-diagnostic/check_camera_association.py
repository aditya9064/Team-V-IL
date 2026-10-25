"""Independent CPU audit of observation collection, not a robot acceptance test.

No simulator imports, timestamp correction, camera-age certification or motion.
Mechanical/raw-ledger acceptance requires a separate audit before admission.
"""
import argparse
import hashlib
import json
import math
from fractions import Fraction
from pathlib import Path

ORDER = ('table', 'side', 'wrist')
AGE_LIMIT = 2 / 60 + 1e-6
SDK = 'fff64d7bb7c6e8bc86c77df300227ba1c70932efac9285c36d43eabe8261d2a8'
PACKET_CONTENT_SHA256 = '768b274f8eb96e268c1154179f4541bbbed94e970c6240f3c32f800b023c1124'
SOURCE_PINS = {'cell.py': 'cfde65b149d4c87add24793734d65d968b79ea70563c169e2f46b458b32a4260',
               'scene_config.py': '6d5d0c7df867986e78ff02b2888236dca6c00cc3bc6b7ecf60f9a49d75facc1b'}
CLOCK_PINS = {'World': 'af07b4e498c25e0df4d149199f123c261ed0541ea25c084a2f88f6d08b461711',
              'SimulationContext': '13f3c98211751bffc14fcd96a23ca7f856abcace50976c27f7b9eccf6c11b9cf'}


def need(ok, text):
    if not ok:
        raise ValueError(text)


def finite(x):
    return type(x) in (int, float) and math.isfinite(x)


def close(a, b, tol=1e-6):
    return finite(a) and finite(b) and abs(a-b) <= tol


def matrix(x):
    need(isinstance(x, list) and len(x) == 4 and all(isinstance(r, list) and len(r) == 4
         and all(finite(v) for v in r) for r in x), 'Invalid matrix')
    return x


def multiply(a, b):
    return [[sum(a[i][k]*b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def normalize(v):
    length = math.sqrt(sum(x*x for x in v))
    need(length > 1e-12, 'Degenerate view vector')
    return [x/length for x in v]


def cross(a, b):
    return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]


def look_at_row(cfg):
    # Independent OpenGL/USD camera basis: +X right, +Y up, +Z backwards.
    z = normalize([a-b for a, b in zip(cfg['eye'], cfg['look_at'])])
    x = normalize(cross(cfg['up'], z)); y = cross(z, x)
    return [x+[0], y+[0], z+[0], list(cfg['eye'])+[1]]


def match_matrix(a, b, label):
    matrix(a); matrix(b)
    need(all(close(x, y) for ra, rb in zip(a, b) for x, y in zip(ra, rb)), label)


def expected_configs(packet):
    # Read only AST literal expressions from the pinned, previously extracted source.
    import ast
    need(hashlib.sha256(json.dumps(packet,sort_keys=True,separators=(',',':')).encode()).hexdigest()
         == PACKET_CONTENT_SHA256, 'Provided source packet content mismatch')
    expected = {}
    for item in packet['scene_config.py']['camera_related_source']:
        node = ast.parse(item['source']).body[0]
        name = node.targets[0].id
        # Fixed source-bound constants read directly from the same installed file.
        def literal(n):
            if isinstance(n, ast.Constant): return n.value
            if isinstance(n, (ast.List, ast.Tuple)): return [literal(v) for v in n.elts]
            if isinstance(n, ast.Name) and n.id in ('ROBOT_PRIM','TABLE_TOP_Z'):
                return {'ROBOT_PRIM':'/World/ER4iA','TABLE_TOP_Z':.9}[n.id]
            if isinstance(n, ast.UnaryOp) and isinstance(n.op, ast.USub): return -literal(n.operand)
            if isinstance(n, ast.BinOp) and isinstance(n.op, ast.Add): return literal(n.left)+literal(n.right)
            if isinstance(n, ast.Call) and isinstance(n.func, ast.Name) and n.func.id=='dict' and not n.args:
                need(all(k.arg is not None for k in n.keywords), 'Expanded config forbidden')
                return {k.arg:literal(k.value) for k in n.keywords}
            raise ValueError('Unsupported config source expression')
        cfg = literal(node.value)
        expected[name.removesuffix('_CAM').lower()] = cfg
    need(set(expected) == set(ORDER), 'Incomplete pinned configuration')
    return expected


def audit_configuration(config, expected):
    need(config['camera_order'] == list(ORDER) and config['resolution'] == [640, 360], 'Wrong view contract')
    need(close(config['physics_dt_s'], 1/60, 1e-12) and close(config['render_dt_s'], 1/30, 1e-12), 'Clock settings changed')
    need(set(config['cameras']) == set(ORDER), 'Unexpected view configuration')
    for name in ORDER:
        actual = config['cameras'][name]; cfg = expected[name]
        # JSON turns tuples into lists; otherwise require exact pinned configuration.
        need(actual['provided_camera_cfg'] == json.loads(json.dumps(cfg)), 'Provided config was changed: '+name)
        suffix = {'table': '/TableCam', 'side': '/SideCam', 'wrist': '/WristCam'}[name]
        need(actual['prim_path'] == cfg['parent']+suffix and actual['resolution'] == [640,360], 'Wrong path or resolution')
        need(close(actual['focal_length_mm'], cfg['focal_length'])
             and close(actual['horizontal_aperture_mm'], cfg['horiz_aperture'])
             and close(actual['vertical_aperture_mm'], cfg['horiz_aperture']*360/640), 'Optics mismatch')
        freq = actual['frequency_hz']
        need(finite(freq) and freq != 0, 'Invalid frequency')
        need(actual['negative_frequency_means_all_frames'] is (freq < 0), 'Frequency semantics mismatch')
        need(actual['camera_dt_s'] is None if freq < 0 else close(actual['camera_dt_s'], 1/freq), 'Frequency/dt mismatch')
        match_matrix(actual['camera_to_parent_row_matrix'], look_at_row(cfg), 'Local camera pose mismatch')
        parent = matrix(actual['parent_to_world_row_matrix'])
        if name != 'wrist':
            match_matrix(parent, [[int(i==j) for j in range(4)] for i in range(4)], 'Fixed camera parent moved')
        match_matrix(actual['camera_to_world_row_matrix'], multiply(actual['camera_to_parent_row_matrix'], parent), 'World pose composition mismatch')
        q = actual['joint_positions_rad_at_configuration']
        need(isinstance(q,list) and len(q)>=6 and all(finite(v) for v in q), 'Missing finite joint readback')


def clock(value):
    need(set(value) == {'step','sim_time'} and type(value['step']) is int and value['step']>=0
         and finite(value['sim_time']) and value['sim_time']>=0, 'Malformed native clock')
    return value


def decode(raw):
    # Independent typed evidence decoder; no shared runtime metadata parser.
    kind = raw['type']
    if 'items' in raw:
        need(raw['truncated'] is False and raw['item_count']==len(raw['items']), 'Truncated native evidence')
        pairs = [(decode(x['key']), decode(x['value'])) for x in raw['items']]
        need(len({x[0] for x in pairs})==len(pairs), 'Duplicate native key')
        return dict(pairs)
    value = raw['value']
    if kind in ('builtins.str',):
        need(type(value) is str, 'Typed string mismatch')
    elif kind in ('builtins.int','numpy.int64','numpy.uint64','numpy.int32'):
        need(type(value) is int, 'Typed integer mismatch')
    elif kind in ('builtins.float','numpy.float64','numpy.float32'):
        need(finite(value), 'Nonfinite native time')
    else:
        raise ValueError('Unsupported native scalar type: '+kind)
    return value


def metadata(cached):
    md = cached['metadata']
    need(md['SDK_camera_source_sha256']==SDK and md['capture_before_conversion'] is True, 'Wrong metadata provenance')
    raw = md['native_metadata']
    need(all(raw[k]['available'] is True for k in ('rendering_frame','rendering_time')), 'Missing native metadata')
    ref = decode(raw['rendering_frame']['raw']); t = decode(raw['rendering_time']['raw'])
    n, d = ref['referenceTimeNumerator'], ref['referenceTimeDenominator']
    need(type(n) is int and type(d) is int and n>=0 and d>0 and finite(t) and t>=0, 'Invalid native reference')
    return Fraction(n,d), t


def image(value):
    need(type(value['available']) is bool, 'Image availability absent')
    if not value['available']:
        return False
    need(value['shape']==[360,640,4] and value['dtype']=='uint8' and value['bytes']==360*640*4, 'Wrong image shape')
    need(isinstance(value['sha256'],str) and len(value['sha256'])==64
         and all(c in '0123456789abcdef' for c in value['sha256']), 'Invalid pixel hash')
    return True


def audit(report, source, packet):
    h = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
    policy = json.loads((source/'camera_association_policy.json').read_text())
    need(report['trial_script_sha256']==h(source/'camera_association_trial.py')==policy['runtime_sha256'], 'Runtime identity mismatch')
    need(report['camera_policy_sha256']==h(source/'camera_association_policy.json'), 'Policy mismatch')
    need(report['camera_API_source_sha256']==SDK and report['clock_API_sources']==CLOCK_PINS, 'SDK mismatch')
    need(all(report['starter_hashes'][k]==v==packet[k]['sha256'] for k,v in SOURCE_PINS.items()), 'Starter mismatch')
    need(report['status']=='OBSERVATION_DIAGNOSTIC_COMPLETE' and report['task']=='camera_association_observation_only', 'Collection did not complete')
    need(not report['part_results'] and not report['motion_sampling']['phases'] and not report['part_order'], 'Unexpected pickup dispatch')
    need(not report['unexpected_contacts'] and not report['contact_monitor_errors'], 'Runtime physical stop/error')
    for key in ('full_assembly_accepted','runtime_placement_completed','camera_freshness_qualified'):
        need(report[key] is False, 'Unqualified positive claim: '+key)
    probe = report['camera_probe']; config = probe['configuration']
    need(probe['helper_sha256']==h(source/'capture_probe.py')==policy['collector_sha256'], 'Collector mismatch')
    need(probe['pixel_age_certified'] is False and probe['pickup_dispatched'] is False, 'Unsafe probe claim')
    audit_configuration(config,expected_configs(packet))
    baseline = clock(config['native_clock_before'])
    need(clock(probe['native_clock_after'])==baseline, 'Whole probe advanced physics')
    need(probe['runtime_state_step_after']==config['runtime_state_step']
         and baseline['step']-config['runtime_state_step']==config['native_minus_runtime_step'], 'Runtime/native clock offset changed')
    expected = []
    for i in range(7):
        if i: expected.append(('render_only_refresh',i,None))
        expected.extend(('capture_transaction',i,c) for c in ORDER)
    records = probe['records']
    need(len(records)==27, 'Incomplete or over-cap collection')
    summaries = {c:[] for c in ORDER}
    for r, key in zip(records,expected):
        need((r['kind'],r['index'],r.get('camera'))==key and r['complete'] is True, 'Event order/cap/failure mismatch')
        need(clock(r['clock_before'])==baseline==clock(r['clock_after']), 'Transaction/render advanced physics')
        if r['kind']!='capture_transaction': continue
        need(r['pixel_age_certified'] is False and r['observation_only'] is True, 'Unsafe capture claim')
        before, after = metadata(r['cached_before']),metadata(r['cached_after'])
        available = [image(r['cached_before']['rgba']),image(r['direct_rgba']),image(r['cached_after']['rgba'])]
        ages = [baseline['sim_time']-before[1],baseline['sim_time']-after[1]]
        summaries[r['camera']].append({'index':r['index'],'native_reference_before':str(before[0]),
            'native_reference_after':str(after[0]),'native_metadata_unchanged_within_transaction':before==after,
            'cached_metadata_ages_s':ages,'cached_metadata_within_original_age_limit':all(0<=x<=AGE_LIMIT for x in ages),
            'all_three_pixel_buffers_available':all(available),
            'all_three_pixel_hashes_equal':all(available) and len({r[k]['rgba']['sha256'] for k in ('cached_before','cached_after')}|{r['direct_rgba']['sha256']})==1})
    return {'collection_structure_verified':True,'per_view':summaries,'original_age_limit_s':AGE_LIMIT,
            'pixel_age_certified':False,'freshness_qualified':False,'pickup_qualified':False,
            'physical_calibration_qualified':False,'mechanical_raw_ledger_audited':False,
            'scope':'Native metadata and buffer association observations only. No actual pixel exposure time, cause, replay-resistant live pair, mechanical or robot acceptance.'}


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('report',type=Path);p.add_argument('--source',type=Path,required=True)
    p.add_argument('--packet',type=Path,required=True);a=p.parse_args()
    print(json.dumps(audit(json.loads(a.report.read_text()),a.source,json.loads(a.packet.read_text())),indent=2))
