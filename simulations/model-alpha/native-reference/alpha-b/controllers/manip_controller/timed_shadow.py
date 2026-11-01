"""Add explicit state time around the unchanged204attitude baseline."""
from attitude_shadow import ShadowAttitude

class TimedShadowAttitude(ShadowAttitude):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs);self.state_read_s=None
    def update(self,sample):
        row=super().update(sample)
        if row['fault'] is None and row['quaternion_WB'] is not None and row['status'] in ['initialized_from_gravity','shadow_attitude']:self.state_read_s=row['read_s']
        return dict(row,state_read_s=self.state_read_s,acquisition_s=None)

def original_projection(row):return {k:v for k,v in row.items() if k not in ['state_read_s','acquisition_s']}
