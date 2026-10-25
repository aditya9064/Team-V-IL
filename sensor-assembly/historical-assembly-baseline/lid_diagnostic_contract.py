"""Exact diagnostic stage schedules and fail-closed evidence contract."""
import math
BASE=['orient','above','approach','side_entry','close','lift','raise_for_transfer','carry','level_held','lower','relax_clamp','square_fingers','release','side_exit','retreat']
CARRIER=['orient','above','approach','side_entry','close','lift','raise_for_transfer','carry','lower','release','side_exit','retreat']
LID=['orient','above','approach','side_entry','close','lift','raise_for_transfer','carry','level_held','precontact','lower','release','side_exit','retreat']
def expected_phases(stage):
 if stage not in ('precontact','seat','release'):raise ValueError('Unknown diagnostic stage')
 lid=LID[:10 if stage=='precontact' else 11 if stage=='seat' else len(LID)]
 return ['1_base_'+p for p in BASE]+['2_carrier_'+p for p in CARRIER]+['3_cover_'+p for p in lid]
def expected_checkpoints(stage):
 labels=['initial']
 for phase in expected_phases(stage):
  labels.append(phase)
  if phase=='1_base_carry':labels.extend(['1_base_held_inspection_a','1_base_held_inspection_b'])
  if phase=='1_base_level_held':labels.extend(['1_base_leveled_inspection_a','1_base_leveled_inspection_b'])
  if phase=='3_cover_carry':labels.extend(['3_cover_held_inspection_a','3_cover_held_inspection_b'])
  if phase=='3_cover_level_held':labels.extend(['3_cover_leveled_inspection_a','3_cover_leveled_inspection_b'])
 if stage=='release':labels.append('final_assembly_inspection')
 return labels
def orientation_for_lid_phase(label,requested,new_spread,planned):
 # Preserve the measured full3D frame only for post-estimation lid phases.
 if label.startswith('3_cover_') and requested is None and new_spread is None and planned is not None:return planned
 return requested

def validate_foot_budget(value,stage='precontact'):
 required={'kind','qualified','fit_foot_height_bounds_mm','calibration_height_error_mm','hold_drift_position_error_mm','maximum_fk_position_error_mm','support_z_error_mm','support_tilt_deg','support_xy_error_mm','fit_position_error_mm','fit_rotation_error_deg','evidence'}
 if not isinstance(value,dict) or set(value)!=required:raise ValueError('Complete explicit precontact budget required')
 if value['kind']!='PRECONTACT_ONLY_SIMULATION' or stage!='precontact':raise ValueError('Final seating/release remains BLOCKED; precontact budget cannot authorize it')
 if value['qualified'] is not True:raise ValueError('Precontact diagnostic budget not qualified')
 bound=value['fit_foot_height_bounds_mm']
 if not isinstance(bound,list) or len(bound)!=4:raise ValueError('Four empirical foot-height bounds required')
 numeric=bound+[value[k] for k in required-{'kind','qualified','fit_foot_height_bounds_mm','evidence'}]
 if any(isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) or v<0 for v in numeric):raise ValueError('Explicit finite nonnegative bounds required')
 if max(bound)>.05 or value['hold_drift_position_error_mm']>.02 or value['maximum_fk_position_error_mm']>.05:raise ValueError('New diagnostic veto budgets exceeded')
 if value['support_z_error_mm']!=.35 or value['support_tilt_deg']!=1 or value['support_xy_error_mm']!=.75:raise ValueError('Original support placement envelope must be retained conservatively')
 if value['fit_position_error_mm']!=.15 or value['fit_rotation_error_deg']!=.25:raise ValueError('Original independent fit pose envelope required for horizontal uncertainty')
 if not isinstance(value['evidence'],dict) or set(value['evidence'])!=required-{'kind','qualified','evidence'}:raise ValueError('Every budget component needs explicit evidence')
 if any(not isinstance(v,str) or not v.strip() for v in value['evidence'].values()):raise ValueError('Missing diagnostic budget evidence')
 return value

def precontact_budget(mesh,b,observed_fk=False,fit_extra_mm=0.):
 from foot_audit import support_geometry,direct_error_budget
 radius=max(math.hypot(v[0],v[1]) for f in support_geometry(mesh) for v in f['corners_mm'])
 part_xy=b['fit_position_error_mm']+2*radius*math.sin(math.radians(b['fit_rotation_error_deg'])/2)+b['maximum_fk_position_error_mm']+b['hold_drift_position_error_mm']
 if not math.isfinite(fit_extra_mm) or fit_extra_mm<0:raise ValueError('Invalid fit axis-rotation budget')
 return direct_error_budget(mesh,max(b['fit_foot_height_bounds_mm'])+fit_extra_mm,b['calibration_height_error_mm'],b['hold_drift_position_error_mm'],b['support_z_error_mm'],b['support_tilt_deg'],0. if observed_fk else b['maximum_fk_position_error_mm'],b['support_xy_error_mm'],part_xy,b['evidence'])
