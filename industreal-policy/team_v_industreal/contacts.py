"""Read-only PhysX contact records. Absence of reports is never separation proof."""
import math


class ContactAudit:
    def __init__(self,maximum_depth_m=0.00025):
        self.maximum_depth_m=maximum_depth_m
        self.allow_table_support=False
        self.reset()

    def reset(self):
        self.allow_table_support=False
        self.max_depth_m=0.0
        self.socket_contact_observed=False
        self.socket_contact_candidates_observed=False
        self.grip_contact_observed=False
        self.point_count=0
        self.header_count=0
        self.errors=[]
        self.unexpected=[]
        self.tensor_frames=0
        self.tensor_points=0
        self.bilateral_grip_frames=0
        self.last_bilateral_grip=False
        self.maximum_pair_force_n={}
        self.maximum_pair_depth_m={}
        self.last_pair_force_world_n={}
        self.force_crosscheck_frames=0
        self.maximum_force_reconstruction_error_n=0.
        self.net_force_coverage_frames=0
        self.maximum_unaccounted_force_n=0.

    def ingest_tensor(self, names, counts, starts, distances, forces, normals=None, matrices=None, net_force=None):
        """Read indexed PhysX contact buffers, never infer contact from proximity.

        Only active entries may be read: unused PhysX buffer rows may contain
        NaNs. Signed scalars are retained for vector reconstruction. Negative
        scalars are accepted only with finite normals and an independently
        queried pair-force matrix that agrees with the reconstructed force.
        Normal forces are queried with the actual physics dt in the caller.
        """
        try:
            if len(names)!=len(counts) or len(names)!=len(starts):
                raise ValueError('Contact tensor pair shape mismatch')
            self.tensor_frames+=1
            pair_forces={}
            pair_vectors={}
            corroborated=normals is not None and matrices is not None
            if corroborated and (len(normals)!=len(forces) or len(matrices)!=len(names)):
                raise ValueError("Contact force corroboration shape mismatch")
            for pair_index,(name,count,start) in enumerate(zip(names,counts,starts)):
                reconstructed=[0.,0.,0.]
                pair_vectors.setdefault(name,[0.,0.,0.])
                pair_forces.setdefault(name,0.0)
                if int(count)!=count or count<0:
                    raise ValueError('Invalid contact tensor count: '+repr((name,count)))
                if count==0 and not corroborated:continue  # Inactive starts may be-1.
                if count and (int(start)!=start or start<0 or start+count>len(distances) or start+count>len(forces)):
                    raise ValueError('Contact tensor index range invalid: '+repr((name,count,start,len(distances),len(forces))))
                for i in range(int(start),int(start+count)):
                    separation=float(distances[i]);force=float(forces[i])
                    if not math.isfinite(separation) or not math.isfinite(force) or (force<0 and not corroborated):
                        raise ValueError('Invalid active contact tensor value: '+repr({'pair':name,'index':i,'count':count,'start':start,'separation_m':separation,'force_n':force}))
                    self.tensor_points+=1
                    self.max_depth_m=max(self.max_depth_m,max(0.0,-separation))
                    self.maximum_pair_depth_m[name]=max(self.maximum_pair_depth_m.get(name,0.),max(0.,-separation))
                    pair_forces[name]+=abs(force)
                    if corroborated:
                        normal=[float(v) for v in normals[i]]
                        if len(normal)!=3 or not all(math.isfinite(v) for v in normal):
                            raise ValueError("Invalid active contact normal")
                        if force and abs(math.sqrt(sum(v*v for v in normal))-1.)>.001:
                            raise ValueError("Active contact normal is not unit length")
                        for axis in range(3):reconstructed[axis]+=force*normal[axis]
                    if name=='FixedAsset':
                        self.socket_contact_candidates_observed=True
                        if abs(force)>.001 or separation<=0:self.socket_contact_observed=True
                    if name in ('JawL','JawR') and (abs(force)>.001 or separation<=0):self.grip_contact_observed=True
                    if name=='Table' and (abs(force)>.001 or separation<=0) and not self.allow_table_support:
                        self.unexpected.append(['HeldAsset','Table'])
                if corroborated:
                    expected=[float(v) for v in matrices[pair_index]]
                    if len(expected)!=3 or not all(math.isfinite(v) for v in expected):
                        raise ValueError("Invalid aggregate contact force")
                    error=math.sqrt(sum((a-b)**2 for a,b in zip(expected,reconstructed)))
                    self.maximum_force_reconstruction_error_n=max(self.maximum_force_reconstruction_error_n,error)
                    tolerance=max(1e-5,.001*math.sqrt(sum(v*v for v in expected)))
                    if error>tolerance:
                        raise ValueError("Contact force reconstruction disagrees with matrix: "+repr((name,error,tolerance,reconstructed,expected)))
                    for axis in range(3):pair_vectors[name][axis]+=expected[axis]
            if corroborated:
                self.force_crosscheck_frames+=1
                self.last_pair_force_world_n=pair_vectors
                if net_force is not None:
                    net_force=[float(v) for v in net_force]
                    if len(net_force)!=3 or not all(math.isfinite(v) for v in net_force):
                        raise ValueError("Invalid total held-part contact force")
                    known=[sum(v[axis] for v in pair_vectors.values()) for axis in range(3)]
                    error=math.sqrt(sum((a-b)**2 for a,b in zip(net_force,known)))
                    self.maximum_unaccounted_force_n=max(self.maximum_unaccounted_force_n,error)
                    tolerance=max(.001,.001*sum(math.sqrt(sum(x*x for x in v)) for v in pair_vectors.values()))
                    if error>tolerance:
                        self.unexpected.append({'unaccounted_force_n':error,'tolerance_n':tolerance,'net_force':net_force,'known_force':known})
                    self.net_force_coverage_frames+=1
                pair_forces={name:math.sqrt(sum(v*v for v in vector)) for name,vector in pair_vectors.items()}
            for name,force in pair_forces.items():
                self.maximum_pair_force_n[name]=max(self.maximum_pair_force_n.get(name,0.),force)
            self.last_bilateral_grip=all(pair_forces.get(jaw,0.)>0.001 for jaw in ('JawL','JawR'))
            self.bilateral_grip_frames+=int(self.last_bilateral_grip)
        except Exception as exc:
            self.errors.append(repr(exc))

    def ingest(self,headers,data,decode):
        try:
            for h in headers:
                self.header_count+=1
                n=int(h.num_contact_data);start=int(h.contact_data_offset)
                if n<0 or start<0 or start+n>len(data): raise ValueError('Contact header index range invalid')
                paths=[str(decode(h.actor0)),str(decode(h.actor1))]
                def role(path):
                    for name in ('Robot','FixedAsset','HeldAsset','Table'):
                        if name in path.split('/'): return name
                    return 'other'
                pair={role(p) for p in paths}
                if not pair & {'Robot','HeldAsset'}: continue
                allowed=pair in ({'Robot'},{'Robot','HeldAsset'},{'HeldAsset','FixedAsset'})
                if n and not allowed: self.unexpected.append(paths)
                for i in range(start,start+n):
                    raw=data[i]
                    separation=float(raw.separation)
                    if not math.isfinite(separation): raise ValueError('Nonfinite contact separation')
                    self.point_count+=1
                    self.max_depth_m=max(self.max_depth_m,max(0.0,-separation))
                    if pair=={'HeldAsset','FixedAsset'}: self.socket_contact_observed=True
                    if pair=={'HeldAsset','Robot'}: self.grip_contact_observed=True
        except Exception as exc:
            self.errors.append(repr(exc))

    @property
    def invalid(self):
        return bool(self.errors or self.unexpected or self.max_depth_m>self.maximum_depth_m)

    def report(self):
        return {'contact_monitor_valid':not self.errors and (self.header_count>0 or self.tensor_frames>0),'socket_contact_observed':self.socket_contact_observed,
                'grip_contact_observed':self.grip_contact_observed,
                'socket_contact_candidates_observed':self.socket_contact_candidates_observed,
                'max_contact_depth_m':self.max_depth_m,'raw_contact_points':self.point_count,
                'raw_contact_headers':self.header_count,'errors':list(self.errors),
                'unexpected_contacts':list(self.unexpected),
                'tensor_frames':self.tensor_frames,'tensor_contact_points':self.tensor_points,
                'bilateral_grip_frames':self.bilateral_grip_frames,
                'bilateral_grip_fraction':self.bilateral_grip_frames/max(1,self.tensor_frames),
                'last_bilateral_grip':self.last_bilateral_grip,
                'maximum_pair_normal_force_n':dict(self.maximum_pair_force_n),
                'maximum_pair_depth_m':dict(self.maximum_pair_depth_m),
                'last_pair_normal_force_world_n':dict(self.last_pair_force_world_n),
                'force_crosscheck_frames':self.force_crosscheck_frames,
                'maximum_force_reconstruction_error_n':self.maximum_force_reconstruction_error_n,
                'net_force_coverage_frames':self.net_force_coverage_frames,
                'maximum_unaccounted_force_n':self.maximum_unaccounted_force_n,
                'no_header_means':'NO_REPORTED_RECORD; never proof of separation'}


def active_prefix_length(counts,starts,capacity):
    """Smallest safe buffer prefix retaining every indexed active contact."""
    if len(counts)!=len(starts) or not isinstance(capacity,int) or capacity<0:
        raise ValueError('Invalid contact buffer shape')
    end=0
    for count,start in zip(counts,starts):
        if not math.isfinite(count) or int(count)!=count or count<0:raise ValueError('Invalid active contact count')
        if not count:continue
        if not math.isfinite(start) or int(start)!=start or start<0 or start+count>capacity:
            raise ValueError('Active contact buffer overrun')
        end=max(end,int(start+count))
    return end
