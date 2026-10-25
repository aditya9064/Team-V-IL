"""Fixed CAD/recipe jaw-opening profile. No object pose/force or truth input."""
import math

def lid_release_samples(duration,physics_dt):
 if not math.isclose(duration,3.,abs_tol=1e-12) or not math.isclose(physics_dt,1/60,abs_tol=1e-12):raise ValueError('Audited lid release requires3s at60Hz')
 return [(i*physics_dt,10*(i/180)**3-15*(i/180)**4+6*(i/180)**5) for i in range(1,181)]

def lid_release_bounds(distance_m):
 if not math.isfinite(distance_m) or not 0<distance_m<.006:raise ValueError('Finite current-jaw travel required')
 return {'duration_s':3.,'commands':180,'profile':'quintic_10u3_15u4_6u5','start_and_end_velocity':0.,'start_and_end_acceleration':0.,'same_jaw_opening_distance_m':distance_m,'maximum_speed_m_s':distance_m*1.875/3,'maximum_acceleration_m_s2':distance_m*(10/math.sqrt(3))/9,'old_duration_s':1.,'old_maximum_speed_m_s':distance_m*1.5,'old_maximum_acceleration_m_s2':distance_m*6,'speed_ratio':1.875/4.5,'acceleration_ratio':(10/math.sqrt(3))/54,'recipe_only':True}
