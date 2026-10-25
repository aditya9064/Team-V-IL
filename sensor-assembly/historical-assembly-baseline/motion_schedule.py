"""Sample a fixed-duration nominal path once per physics step."""
import math

def motion_samples(duration, physics_dt):
    if not all(math.isfinite(v) and v > 0 for v in (duration, physics_dt)):
        raise ValueError('Positive finite duration and physics_dt required')
    count=round(duration/physics_dt)
    if count < 1 or not math.isclose(count*physics_dt,duration,abs_tol=1e-9):
        raise ValueError('Duration must contain a whole number of physics steps')
    return [(i*physics_dt,(i/count)**2*(3-2*i/count)) for i in range(1,count+1)]
