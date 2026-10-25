"""Prospectively fixed diagnostic policy; historical criteria are not rewritten."""
PHASES=['3_cover_'+s for s in ['orient','above','approach','side_entry','close','lift','raise_for_transfer','carry','level_held','precontact','lower','release','side_exit','retreat']]
DURATIONS=[1.,2.,2.,1.5,3.,1.5,1.5,2.,2.,4.,2.,3.,1.5,2.]
CONTRACT={
 'schema':'ORIGINAL_LID_OBSERVED_PLACEMENT_V58B',
 'scope':'Isolated original-lid pickup, transfer, RGB alignment, seating/release diagnostic; dynamic base/carrier/loosePCB are prepositioned fixtures, not robot-placed',
 'human_experiment_change':'Record .020mm holding movement and derived certification screens; do not stop solely for them. Observe actual placement. Original model/setup, tracking, contact, inventory, loosePCB and actual placement/release gates remain.',
 'oracle_precision_policy':'Historical .15mm position/.25deg orientation/.05mm corrected-foot comparisons against simulator truth are measurements ONLY, never target corrections or placement admission',
 'old_hold_mm':.020,'old_hold_policy':'MEASURE_ONLY_NOT_TERMINATING',
 'derived_clearance_budget_policy':'ADVISORY_NOT_PREDICTIVELY_CERTIFIED',
 'all_CAD_fingers_physics_materials_mass_unchanged':True,
 'truth_targets_or_adaptive_wait':False,
 'target_inputs':['RGB','fixed camera calibration','measured robot FK','original CAD and static fixture recipe'],
 'target_validity_stops':['missing/nonfinite evidence','RGB image fit, finite/calibrated target invalid','RGB two-frame disagreement','IK/command residual or measured endpoint invalid'],
 'process_stops':['lost grip original2mm/5deg/3deg','original raw contact classification and0.250mm depth','loosePCB or fixture inventory outside original limits','actual lower alignment outside original placement limits','original0.250mm fixed-reference release excursion','actual jaws not open or final placement invalid'],
 'hard_limits':{'TCP_mm':2.,'tool_spread_deg':1.,'camera_speed_rad_s':.02,'consecutive_samples':6,'raw_contact_depth_mm':.25,'release_3D_excursion_mm':.25,'placement_XY_mm':.75,'placement_Z_mm':.35,'placement_tilt_yaw_deg':1.},
 'full_assembly_accepted':False,'physical_hardware_qualification':False,
 'final_support_evidence':'All four actual feet within assigned actual pads, original final pose limits throughout existing120-step inspection, positive normal-component bearing records for each actual foot/pad in both halves, actual source fingers geometrically clear, original release reference unchanged; record trajectory/impulses, do not infer static equilibrium; missing evidence is INCONCLUSIVE',
 'video':'Actual recording through outcome stays on Rosie; failures retained'
}
