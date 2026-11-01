"""Manufactured228row composition only;no hardware driver or register semantic claim."""
from phase import CounterPhaseModel,integer
class ManufacturedBurstCounter:
    def __init__(self,epoch=1,baseline_transaction_sequence=0,**clock):
        if not integer(baseline_transaction_sequence) or baseline_transaction_sequence<0:raise ValueError('declared baseline transaction sequence')
        self.phase=CounterPhaseModel(epoch=epoch,**clock);self.last_transaction=baseline_transaction_sequence;self.total=0;self.anchor_sequence=0;self.fault=None
    def update(self,row,minimum_elapsed_ticks=0,maximum_elapsed_ticks=None,provenance='manufactured'):
        result=None
        if self.fault is None:
            try:
                if provenance!='manufactured' or not isinstance(row,dict):raise ValueError('manufactured burst evidence only')
                if row.get('fault') is not None or row.get('grammar_compatible') is not True or row.get('model_owner_bytes_match') is not True:raise ValueError('burst grammar/owner evidence unavailable')
                if row.get('hardware_configuration_qualified') is not False or row.get('hardware_driver_executed') is not False:raise ValueError('model-only composition')
                if not integer(row.get('current_model_epoch')) or row.get('current_model_epoch')!=self.phase.epoch or row.get('pending_model_epoch') is not None:raise ValueError('pending/changed epoch')
                seq=row.get('transaction_sequence')
                if not integer(seq) or seq!=self.last_transaction+1:raise ValueError('transaction sequence gap/duplicate')
                frames=[e for e in row['events'] if e['kind'] in ['accel','gyro','gyro_accel']]
                if any(e['kind']!='gyro_accel' or not integer(e['model_epoch_id']) or e['model_epoch_id']!=self.phase.epoch or e['model_owner_bytes_match'] is not True for e in frames):raise ValueError('single-period combined-frame model only')
                if not integer(row['regular_frames']) or row['regular_frames']!=len(frames):raise ValueError('frame count mismatch')
                ticks=[e['sensor_time_ticks'] for e in row['events'] if e['kind']=='sensor_time']
                if len(ticks)>1 or (ticks[0] if ticks else None)!=row['observed_terminal_read_time_ticks']:raise ValueError('read-time control mismatch')
                total=self.total+len(frames)
                if ticks:
                    result=self.phase.observe(ticks[0],total,self.anchor_sequence+1,self.phase.epoch,minimum_elapsed_ticks,maximum_elapsed_ticks,provenance)
                    if result['fault'] is not None:raise ValueError('conditional counter phase: '+result['fault'])
                    self.anchor_sequence+=1
                self.total=total;self.last_transaction=seq
            except (ValueError,TypeError,KeyError) as error:self.fault=type(error).__name__+':'+str(error);result=None
        return dict(status='latched_unavailable' if self.fault else 'conditional_anchor_observed' if result is not None else 'awaiting_read_time_anchor',fault=self.fault,cumulative_manufactured_combined_frames=self.total,last_accepted_transaction_sequence=self.last_transaction,conditional_phase=result,acquisition_s=None,conversion_freshness_proven=False,host_device_clock_mapping_qualified=False,hardware_driver_executed=False,consumer_reset_authorized=False,usable_for_feedback=False)
