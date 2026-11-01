"""Transport actual API snapshots without inventing acquisition time or an estimate."""
import copy,math
class SnapshotBuffer:
    def __init__(self):self.pending=[];self.last_producer=0.;self.last_host=0.;self.latest=None;self.fault=None
    def push(self,row):
        if self.fault is not None:return
        try:
            t=row['producer_read_s']
            if set(row)!={'producer_read_s','samples'} or not isinstance(row['samples'],dict):raise ValueError('producer schema')
            if not isinstance(t,(int,float)) or isinstance(t,bool) or not math.isfinite(t) or abs(t-self.last_producer-.005)>1e-9:raise ValueError('producer cadence')
            if t<self.last_host-1e-9:raise ValueError('producer read predates host')
            self.pending.append(copy.deepcopy(row));self.last_producer=t
        except (ValueError,TypeError,KeyError) as e:self.fault=type(e).__name__+':'+str(e)
    def read(self,t):
        batch=[]
        if self.fault is None:
            if not isinstance(t,(int,float)) or isinstance(t,bool) or not math.isfinite(t) or abs(t-self.last_host-.008)>1e-9:self.fault='ValueError:host cadence'
            elif self.last_producer>t+1e-9:self.fault='ValueError:future producer read'
            else:
                batch=self.pending;self.pending=[];self.last_host=t
                if batch:self.latest=batch[-1]['producer_read_s']
        safe=t if isinstance(t,(int,float)) and not isinstance(t,bool) and math.isfinite(t) else None
        return dict(host_read_s=safe,status='latched_transport_unavailable' if self.fault else 'queued_api_snapshots' if batch else 'no_new_snapshot',fault=self.fault,batch=batch,latest_producer_read_s=self.latest,api_read_age_s=None if self.latest is None or safe is None else safe-self.latest,pending=len(self.pending),acquisition_s=None,conversion_freshness_proven=False,hardware_driver_executed=False,usable_for_feedback=False)
