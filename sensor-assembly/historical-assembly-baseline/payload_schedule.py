"""Fixed planned tray-transfer profile; no truth or live object state inputs."""
import math

def payload_carry_samples(duration,physics_dt):
    if not math.isclose(duration,4.,abs_tol=1e-9) or not math.isclose(physics_dt,1/60,abs_tol=1e-12):raise ValueError('Audited payload transfer is4s at60Hz only')
    return [(i*physics_dt,10*(i/240)**3-15*(i/240)**4+6*(i/240)**5) for i in range(1,241)]

def profile_bounds(distance_m):
    if not math.isfinite(distance_m) or distance_m<=0:raise ValueError('Positive finite transfer distance required')
    return {'duration_s':4.,'peak_speed_m_s':distance_m*1.875/4,'peak_acceleration_m_s2':distance_m*(10/math.sqrt(3))/16,'endpoint_acceleration_m_s2':0.,'old_duration_s':2.,'old_peak_acceleration_m_s2':distance_m*6/4,'acceleration_ratio':(10/math.sqrt(3))/24}
