"""Prospective logging order only: collect the completed step's PCB guard first.

No physics, control, data repair, tolerance change or exception suppression.
The existing contact stop keeps priority over a simultaneous payload failure.
"""


def record_payload_then_check_contacts(report, sample_payload, check_contacts):
    try:
        score = sample_payload()
        previous = report.get('payload_motion_audit', {
            'samples': 0, 'maximum_xy_mm': 0., 'maximum_z_mm': 0.,
            'maximum_relative_tilt_deg': 0., 'maximum_relative_yaw_deg': 0.,
        })
        # A failed metric read must not leave a partly counted/default-valued
        # terminal record behind. Commit the complete aggregate atomically.
        audit = dict(previous)
        audit['samples'] += 1
        for key, resultkey in [('xy_mm', 'maximum_xy_mm'), ('z_mm', 'maximum_z_mm'),
                               ('tilt_deg', 'maximum_relative_tilt_deg'), ('yaw_deg', 'maximum_relative_yaw_deg')]:
            audit[resultkey] = max(audit[resultkey], score[key])
        report['payload_motion_audit'] = audit
    except Exception:
        # A recorded contact stop was previously checked before any PCB work.
        # Preserve its priority even if the read-only PCB sample also errors.
        check_contacts()
        raise
    check_contacts()
    return score
