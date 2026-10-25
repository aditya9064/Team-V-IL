"""In-memory raw contact telemetry. No simulator imports, mutations or control outputs.

Raw ordered vectors are never reoriented. Callback time is not a generation time.
No header is evidence of NO_REPORTED_RECORD, never evidence of physical separation.
"""
import math

SCHEMA='PCB_CARRIER_CONTACT_TRACE_V1'
ABSENCE='NO_REPORTED_RECORD; physical contact/load state unknown'

def integer(value):
    if isinstance(value,bool):raise ValueError('Boolean is not an integer identifier')
    result=int(value)
    if result!=value:raise ValueError('Non-integral identifier/count')
    return result

def finite(value):
    result=float(value)
    if not math.isfinite(result):raise ValueError('Non-finite telemetry scalar')
    return result

def vector(value):
    result=[finite(x) for x in value]
    if len(result)!=3:raise ValueError('Expected xyz vector')
    return result

def descendant(path,root):return path==root or path.startswith(root+'/')

def selected_pair(actors,colliders,pcb_root,carrier_roots):
    """Exact root/subtree matching, both orders. Roles never replace identities."""
    if len(actors)!=2 or len(colliders)!=2:raise ValueError('Ordered pair required')
    for pcb_index,carrier_index in ((0,1),(1,0)):
        pcb=descendant(actors[pcb_index],pcb_root) or descendant(colliders[pcb_index],pcb_root)
        carriers=[root for root in carrier_roots if descendant(actors[carrier_index],root) or descendant(colliders[carrier_index],root)]
        if pcb and len(carriers)==1:return {'pcb_index':pcb_index,'carrier_index':carrier_index,'carrier_root':carriers[0]}
    return None

def optional_integer(obj,name):
    if not hasattr(obj,name):return {'available':False,'value':None}
    return {'available':True,'value':integer(getattr(obj,name))}

def ordered_identity(header,decode_path,collider_roles):
    actor_ids=[integer(header.actor0),integer(header.actor1)]
    collider_ids=[integer(header.collider0),integer(header.collider1)]
    actors=[str(decode_path(x)) for x in actor_ids];colliders=[str(decode_path(x)) for x in collider_ids]
    if any(not x.startswith('/') for x in actors+colliders):raise ValueError('Non-absolute raw contact path')
    return {'actor_ids_decimal':[str(x) for x in actor_ids],'collider_ids_decimal':[str(x) for x in collider_ids],
            'actors':actors,'colliders':colliders,'roles':[None if collider_roles.get(x) is None else list(collider_roles[x]) for x in colliders]}

def serialize_header(header,data,identity,pair,callback_sequence,header_ordinal,step,phase,callback_time):
    count=integer(header.num_contact_data);offset=integer(header.contact_data_offset)
    if count<0 or (count and offset<0):raise ValueError('Invalid raw contact slice')
    event=getattr(header,'type',None)
    if event is None:raise ValueError('Missing raw header.type')
    event_code=integer(int(event))
    points=[]
    for index in range(offset,offset+count):
        raw=data[index]
        points.append({'raw_data_index':index,'position_world_m':vector(raw.position),'normal_world_raw':vector(raw.normal),
                       'impulse_Ns_raw':vector(raw.impulse),'separation_m':finite(raw.separation),
                       'separation_mm':finite(raw.separation)*1000,
                       'optional_fields':{name:optional_integer(raw,name) for name in ('face_index0','face_index1','material0','material1')}})
    return dict(identity,selection=pair,callback_sequence=callback_sequence,header_ordinal=header_ordinal,state_step=integer(step),
                phase=str(phase),callback_world_time_s=finite(callback_time),event_type_code=event_code,event_type_repr=repr(event),
                event_type_name={'available':hasattr(event,'name'),'value':str(event.name) if hasattr(event,'name') else None},
                stage_id=optional_integer(header,'stage_id'),contact_data_offset=offset,num_contact_data=count,points=points)

class PayloadContactTrace:
    """All public hooks isolate telemetry errors from original physics/control.

    No buffer cap, decimation, last-N retention, force threshold or file I/O.
    The parent report holds this object by reference; existing persist calls save it.
    """
    def __init__(self,pcb_root,carrier_roots):
        self.pcb_root=str(pcb_root);self.carrier_roots=list(carrier_roots);self.active=False;self.current=None
        self.value={'schema':SCHEMA,'scope':'TELEMETRY ONLY: first active carrier phase step through original diagnostic termination; earlier base and reset history unrecorded',
                    'pcb_root':self.pcb_root,'carrier_roots':self.carrier_roots,'complete':True,'errors':[],
                    'absence_semantics':ABSENCE,'callback_time_semantics':'raw world.current_time read in callback; NOT a contact-generation timestamp; pre/post bracket retained',
                    'vector_semantics':'SDK raw ordered normal and impulse; no flips, abs, sums, force conversion or asserted force-on-PCB sign',
                    'sampling':'every selected header and every indexed point; zero-point headers retained; no truncation or decimation',
                    'physics_or_target_use':False,'callback_count':0,'header_count_total':0,'selected_header_count':0,'selected_point_count':0,'rows':[]}

    def error(self,where,exc,**context):
        self.value['complete']=False;self.value['errors'].append(dict(where=where,error_type=type(exc).__name__,message=str(exc),**context))

    def note_existing_snapshot(self,snapshot):
        if not self.active:return
        try:self.value['last_existing_snapshot']=snapshot
        except Exception as exc:self.error('note_existing_snapshot',exc)

    def begin_step(self,snapshot,diagnostic_index,phase,activate):
        try:
            if not self.active and not activate:return
            if self.current is not None:raise ValueError('Previous telemetry step unclosed')
            self.active=True
            step=integer(snapshot['step']);time=finite(snapshot['sim_time'])
            if not self.value['rows']:self.value['first_step']=step;self.value['first_pre_step_time_s']=time
            row={'state_step':step,'phase':str(phase),'pre_step_time_s':time,'post_step_time_s':None,'closed':False,
                 'pre_step_snapshot_link':{'collection':'payload_contact_pose_trace','index':integer(diagnostic_index),'step':step,'sim_time':time},
                 'callback_count':0,'header_count_total':0,'pcb_carrier_header_count':0,'pcb_carrier_point_count':0,
                 'reported_record_state':'NO_REPORTED_RECORD','callbacks':[],'headers':[]}
            self.value['rows'].append(row);self.current=row
        except Exception as exc:self.error('begin_step',exc)

    def begin_callback(self,step,phase,callback_time):
        if not self.active:return None
        try:
            row=self.current if self.current is not None else (self.value['rows'][-1] if self.value['rows'] else None)
            if row is None or row['state_step']!=step:raise ValueError('Callback outside matching physics bracket')
            self.value['callback_count']+=1;row['callback_count']+=1
            token={'sequence':self.value['callback_count'],'state_step':integer(step),'phase':str(phase),'callback_world_time_s':finite(callback_time() if callable(callback_time) else callback_time),
                   'row_index':len(self.value['rows'])-1,'header_count_total':0,'pcb_carrier_header_count':0,'pcb_carrier_point_count':0,'closed':False}
            row['callbacks'].append(token);return token
        except Exception as exc:self.error('begin_callback',exc,state_step=step);return None

    def observe_header(self,token,header,data,decode_path,collider_roles):
        if token is None:return
        try:
            row=self.value['rows'][token['row_index']]
            ordinal=token['header_count_total'];token['header_count_total']+=1;row['header_count_total']+=1;self.value['header_count_total']+=1
            identity=ordered_identity(header,decode_path,collider_roles)
            pair=selected_pair(identity['actors'],identity['colliders'],self.pcb_root,self.carrier_roots)
            if pair is None:return
            record=serialize_header(header,data,identity,pair,token['sequence'],ordinal,token['state_step'],token['phase'],token['callback_world_time_s'])
            row['headers'].append(record);row['reported_record_state']='REPORTED_RECORDS'
            for container,hkey,pkey in ((token,'pcb_carrier_header_count','pcb_carrier_point_count'),(row,'pcb_carrier_header_count','pcb_carrier_point_count'),(self.value,'selected_header_count','selected_point_count')):
                container[hkey]+=1;container[pkey]+=record['num_contact_data']
        except Exception as exc:self.error('observe_header',exc,callback_sequence=token.get('sequence'),header_ordinal=token.get('header_count_total',0)-1,state_step=token.get('state_step'))

    def end_callback(self,token):
        if token is None:return
        try:token['closed']=True
        except Exception as exc:self.error('end_callback',exc)

    def end_step(self,step,post_time):
        if not self.active:return
        try:
            if self.current is None or self.current['state_step']!=step:raise ValueError('Unmatched physics bracket end')
            self.current['post_step_time_s']=finite(post_time);self.current['closed']=True;self.value['last_step']=integer(step);self.value['last_post_step_time_s']=finite(post_time);self.current=None
        except Exception as exc:self.error('end_step',exc,state_step=step)

    def finalize(self,original_status,original_error):
        try:
            self.value['original_runtime_status']=str(original_status);self.value['original_runtime_error']=original_error
            self.value['capture_scope_reached']=self.active
            if self.current is not None:self.error('finalize',ValueError('Unclosed physics bracket'),state_step=self.current['state_step'])
            if any(not token['closed'] for row in self.value['rows'] for token in row['callbacks']):self.error('finalize',ValueError('Unclosed callback'))
        except Exception as exc:self.error('finalize',exc)
